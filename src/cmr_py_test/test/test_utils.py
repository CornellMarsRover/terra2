from itertools import chain
import rclpy as ros
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription, LaunchService
import launch
import launch.event_handler
from rclpy.node import Node as RclpyNode
import os
import toml
import threading as thread
from cmr_msgs.srv import ActivateNode, DeactivateNode
from lifecycle_msgs.srv import GetState
from datetime import datetime
import signal
import multiprocessing as mp


def run_launch_file(package: str, launch_file: str):
    os.system(f"ros2 launch {package} {launch_file}")


def make_fabric_node(
    package: str = "",
    executable: str = "",
    node_name: str = "",
    namespace="rover",
    config_path="",
    config_string="",
    extra_parameters: dict = dict(),
) -> LaunchNode:
    """
    Creates a fabric node with the given parameters

    Args:
        package (str): The package name to find the node executable in.
            Will be overriden by package name in the toml if one is specified
        executable (str): The name of the node executable.
            Will be overriden by executable name in the toml if one is specified
        node_name (str): The name of the node.
            Will be overriden by node name in the toml if one is specified
        namespace (str): The namespace of the node
        config_path (str): The path to the toml config file. Can be empty if config_string is used
        config_string (str): The toml config string. Can be empty if config_path is used
        extra_parameters (dict): Any extra parameters to add to the node

    """
    config = dict()
    if config_string == "":
        config = toml.load(config_path)
    else:
        config = toml.loads(config_string)

    package = config["package"] if "package" in config else package
    executable = config["executable"] if "executable" in config else executable
    name = config["name"] if "name" in config else node_name

    config_map = {
        "config_path": config_path,
        "config_data": config_string,
        "composition_ns": namespace,
    }
    config_map.update(extra_parameters)

    return LaunchNode(
        package=package,
        executable=executable,
        name=name,
        exec_name=name,
        parameters=[config_map],
    )


def make_node(
    node_name: str,
    package_name: str,
    executable_name: str,
    namespace: str,
    parameters: dict,
    **kwargs,
) -> LaunchNode:
    """
    Creates a node with the given parameters

    Args:
        node_name (str): The name of the node
        package_name (str): The name of the package the node is in
        executable_name (str): The name of the node executable
        namespace (str): The namespace to run the node in
        parameters (dict): The parameters to pass to the node
        **kwargs: Any extra arguments to pass to the node

    """
    return LaunchNode(
        name=node_name,
        package=package_name,
        executable=executable_name,
        namespace=namespace,
        parameters=[parameters],
        **kwargs,
    )


class NodeLauncher:
    """
    A class that handles initializing and shutting down ros2 for each test
    and launching all nodes needed for a test.

    We assume tests are run sequentially
    """

    def __init__(self):
        self.child_pid = 0

    def __enter__(self):
        self.namespace = f"rover_py_test_{int(datetime.utcnow().timestamp() * 1000)}"
        # self.executor.add_node(
        self.nodes = [
            LaunchNode(
                package="cmr_fabric",
                executable="fault_handler",
                namespace=self.namespace,
            ),
            LaunchNode(
                package="cmr_fabric",
                executable="lifecycle_manager",
                namespace=self.namespace,
            ),
        ]

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        ros.shutdown()
        if self.child_pid != 0:
            os.kill(self.child_pid, signal.SIGTERM)
            print("Shutting down pytest")

    def get_namespace(self):
        return self.namespace

    def launch_nodes(self, *nodes: LaunchNode):
        """
        Launches the given nodes in a new thread under the generated namespace
        This method must only be called ONCE per test
        """

        for node in nodes:
            node.namespace = self.namespace
            self.nodes.append(node)
        ld = LaunchDescription(self.nodes)
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnShutdown(
                    on_shutdown=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
        )
        ls = LaunchService()
        ls.include_launch_description(ld)
        pid = os.fork()
        if pid == 0:
            print("Launching from child process")

            def sig_handler(signum, frame):
                print("Shutting down launch")
                for child in mp.active_children():
                    print(f"Shutting down child {child.pid}")
                    child.terminate()
                ls.shutdown()
                print("Called shutdown on launch service")

            signal.signal(signal.SIGTERM, sig_handler)
            ls.run()
            print("Launch finished")
            os._exit(0)
        else:
            self.child_pid = pid


def cmr_node_test(test_func):
    """
    A decorator that should be used for all CMR node tests
    The decorated test should take a single argument, which is the node launcher to
    start all necessary nodes
    """

    def wrapper():
        ros.init()
        with NodeLauncher() as launcher:
            test_func(launcher)

    return wrapper


def call_service_sync(service_type: type, service_name: str, request, timeout_sec=5.0):
    """
    Calls a service synchronously, blocking the calling thread until the service returns

    Args:
        service_type (type): The type of the service
        service_name (str): The name of the service
        request (type): The request to send to the service
        timeout_sec (float): The timeout in seconds to wait for the service to respond
            If `None`, the function will block until its ready
    Returns:
        the response of the service
    Raises:
        Exception: If the service does not become available within the timeout
    """

    node = RclpyNode(f"cmr_py_test_client_{int(datetime.utcnow().timestamp() * 1000)}")
    client = node.create_client(service_type, service_name)

    connection_tries = 0
    while (
        timeout_sec is None or connection_tries < timeout_sec
    ) and not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(
            f"service {service_name} not available, waiting again..."
        )
        connection_tries += 1

    if client.service_is_ready():
        future = client.call_async(request)
        ros.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        else:
            raise Exception(f"Service {service_name} timed out")
    else:
        raise Exception(f"Service {service_name} not available")


def change_fabric_node_lifecycle(node_name: str, srv_type: type, namespace: str):
    """
    Activates or deactivates a fabric node via the lifecycle manager

    Args:
        node_name (str): The name of the node to activate/deactivate
        srv_type (type): The type of service to use. Either ActivateNode or DeactivateNode
    Returns:
        the response of the specified service
    """
    service_name = (
        f"/{namespace}/lifecycle/activate"
        if srv_type == ActivateNode
        else f"/{namespace}/lifecycle/deactivate"
    )
    request = srv_type.Request()
    request.node_name = node_name
    return call_service_sync(srv_type, service_name, request)


def activate_fabric_node(node_name: str, namespace: str):
    """
    Activates a fabric node via the lifecycle manager

    Args:
        node_name (str): The name of the node to activate
        namespace (str): The namespace of the node
    Returns:
        the response of the ActivateNode service
    """
    return change_fabric_node_lifecycle(node_name, ActivateNode, namespace)


def deactivate_fabric_node(node_name: str, namespace: str):
    """
    Deactivates a fabric node via the lifecycle manager

    Args:
        node_name (str): The name of the node to deactivate
        namespace (str): The namespace of the node
    Returns:
        the response of the DeactivateNode service
    """
    return change_fabric_node_lifecycle(node_name, DeactivateNode, namespace)


def get_lifecycle_state(node_name) -> str:
    """
    Gets the current lifecycle state of a node

    Args:
        node_name (str): The name of the node to get the state of
    Returns:
        the current lifecycle state of the node
    """
    return call_service_sync(
        GetState, f"/{node_name}/get_state", GetState.Request()
    ).current_state.label
