import rclpy as ros
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription, LaunchService
import sys
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.substitutions import FindExecutable
from launch.event_handlers import OnProcessStart
from subprocess import check_output
from rclpy.node import Node as RclpyNode
import os
from cmr_msgs.srv import ActivateNode, DeactivateNode
from lifecycle_msgs.srv import GetState
from datetime import datetime
import signal
import toml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv._trigger import Trigger

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import time


def make_launch_file(package: str, launch_file: str, **kwargs):
    """
    Creates a LaunchDescription for a launch file to be run by the NodeLauncher
    This will essentially launch all the nodes of the launch file

    Args:
        package (str): The name of the package the launch file is in
        launch_file (str): The name of the launch file
        **kwargs: Any extra arguments to pass to the launch file
    """

    pkg_dir = get_package_share_directory(package)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", launch_file)),
        **kwargs,
    )


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
    Creates a fabric node that can be launched by the NodeLauncher with the given parameters

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


def config_fabric_nodes(*node_configs: str) -> list:
    """
    Constructs multiple fabric node launch descriptions from toml files or
    toml strings ONLY

    Args:
        *node_configs (str): The paths to the toml files or the toml strings
    Returns:
        A list of launch descriptions for the nodes
    See Also:
        make_fabric_node
    """
    nodes = []
    for node in node_configs:
        if os.path.isfile(node):
            nodes.append(make_fabric_node(config_path=node))
        else:
            nodes.append(make_fabric_node(config_string=node))
    return nodes


def make_node(
    node_name: str,
    package_name: str,
    executable_name: str,
    namespace: str = "rover",
    parameters: dict = {},
    **kwargs,
) -> LaunchNode:
    """
    Creates a node that can be launched by the NodeLauncher with the given parameters

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

    def startup(self):
        """
        Initializes the test namespace and node list

        Must be called before `launch_nodes`
        """

        self.namespace = f"rover_py_test_{int(datetime.utcnow().timestamp() * 1000)}"
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

    def __enter__(self):
        self.startup()
        return self

    def cleanup(self):
        """
        Cleans up the launch service

        Must be called after `launch_nodes`
        """

        if self.child_pid != 0:
            os.kill(self.child_pid, signal.SIGINT)

    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()

    def get_namespace(self):
        return self.namespace

    def __add_wait_action(self, ld: LaunchDescription):
        """
        Adds an event handler to listen for the last node to start
        and then write to the service to the parent process to signal the test that it can start
        """
        process = ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    f"/{self.namespace}/launch_wait ",
                    "std_srvs/srv/Trigger",
                ],
            ],
            shell=True,
        )
        ev = RegisterEventHandler(
            OnProcessStart(
                target_action=self.nodes[-1],
                on_start=[process],
            )
        )
        self.nodes.append(process)
        self.trigger_process = process
        ld.add_action(ev)

        return ld

    def __wait_for_launch_process(self):
        """
        Waits for the child process to make a service call to notify the
        parent that it is ready
        """

        def cb(req, resp):
            resp.success = True
            resp.message = "Test ready"
            return resp

        print("Waiting for launch process to start")
        with ServiceListener(Trigger, f"/{self.namespace}/launch_wait", cb) as serv:
            serv.wait_for_msg()
        print("Test start!")

    def __launch_nodes(self, ld: LaunchDescription):
        print("Launching from child process")

        ld = self.__add_wait_action(ld)
        ls = LaunchService()
        ls.include_launch_description(ld)

        def sig_handler(signum, frame):
            print("Shutting down launch")
            print("Called shutdown on launch service")
            for node in self.nodes:
                node_pid = node.process_details["pid"]
                try:
                    os.kill(node_pid, signal.SIGINT)
                except ProcessLookupError:
                    if node != self.trigger_process:
                        print(
                            f"WARNING: Process {node_pid} already dead. It probably crashed during testing",
                            file=sys.stderr,
                        )
                        # Do not emit warning for the trigger process
            ls.shutdown()

        signal.signal(signal.SIGINT, sig_handler)
        ls.run()
        # Run blocks and needs to run on the main thread
        # so we fork and run it in the child process
        print("Launch finished")

    def launch_nodes(self, *nodes):
        """
        Launches the given nodes in a new thread under the generated namespace
        This method must only be called ONCE per test

        Args:
            *nodes: The nodes, launch files, etc. to launch
        """

        for node in nodes:
            node.namespace = self.namespace
            self.nodes.append(node)
        ld = LaunchDescription(self.nodes)
        pid = os.fork()
        if pid == 0:
            self.__launch_nodes(ld)
            os._exit(0)
        else:
            self.child_pid = pid
            # Forking and launching takes time, so we wait until the last process
            # has started before returning
            self.__wait_for_launch_process()


def cmr_node_test(nodes: list):
    """
    A decorator that should be used for all CMR node tests
    The decorated test should take a single argument, which is the node launcher to
    start all necessary nodes

    The test will wait for the node processes to start before running

    Args:
        nodes (list): A list of nodes to launch

    Example:
    ```
        @cmr_node_test([make_fabric_node(config_string=static_config)])
        def test_demo_node(namespace: str):
            assert get_lifecycle_state("py_test_demo") == "unconfigured"
            activate_fabric_node("py_test_demo", namespace)
            assert get_lifecycle_state("py_test_demo") == "active"
    ```
    """

    def decorator(test_func):
        def wrapper(*args, **kwargs):
            with NodeLauncher() as launcher:
                launcher.launch_nodes(*nodes)
                test_func(*args, namespace=launcher.get_namespace(), **kwargs)

        return wrapper

    return decorator


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
        node.destroy_node()
        if future.done():
            return future.result()
        else:
            raise Exception(f"Service {service_name} timed out")
    else:
        node.destroy_node()
        raise Exception(f"Service {service_name} not available")


def change_fabric_node_lifecycle(
    node_name: str, srv_type: type, namespace: str, cleanup: bool = False
):
    """
    Activates or deactivates a fabric node via the lifecycle manager

    Args:
        node_name (str): The name of the node to activate/deactivate
        srv_type (type): The type of service to use. Either ActivateNode or DeactivateNode
        namespace (str): The namespace of the lifecycle manager
        cleanup (bool): If true and deactivation, the node will be cleaned up
    Returns:
        the response of the specified service
    """
    service_name = ""
    if srv_type == ActivateNode:
        service_name = f"/{namespace}/lifecycle/activate"
    elif cleanup:
        service_name = f"/{namespace}/lifecycle/cleanup"
    else:
        service_name = f"/{namespace}/lifecycle/deactivate"
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


def cleanup_fabric_node(node_name: str, namespace: str):
    """
    Cleans up a fabric node via the lifecycle manager

    Args:
        node_name (str): The name of the node to cleanup
        namespace (str): The namespace of the node
    Returns:
        the response of the CleanupNode service
    """
    return change_fabric_node_lifecycle(node_name, DeactivateNode, namespace, True)


def reconfigure_fabric_node(node_name: str, namespace: str):
    """
    Reconfigures a fabric node via the lifecycle manager by cleaning it up
    and activating it.

    Reconfiguring a node allows it to reread its toml configuration file

    Args:
        node_name (str): The name of the node to reconfigure
        namespace (str): The namespace of the node
    Returns:
        the response of the ReconfigureNode service
    """
    return (
        cleanup_fabric_node(node_name, namespace).success
        and activate_fabric_node(node_name, namespace).success
    )


def get_lifecycle_state(node_name) -> str:
    """
    Gets the current lifecycle state of a node

    Args:
        node_name (str): The name of the node to get the state of
    Returns:
        the current lifecycle state string of the node
    """
    return call_service_sync(
        GetState, f"/{node_name}/get_state", GetState.Request()
    ).current_state.label


def setup_module():
    """
    Setup module for all tests in this module
    """
    ros.init()


def teardown_module():
    """
    Teardown module for all tests in this module
    """
    ros.shutdown()


class CMRTestFixture:
    """
    A base class for CMR node tests that use the same nodes for all tests
    This class will start the nodes once for all tests in the class
    To use this, set the `nodes` class variable to a list of nodes to start

    All tests will wait for the node processes to start before running
    """

    nodes = []
    launcher = NodeLauncher()

    @classmethod
    def get_namespace(cls):
        """
        Gets the namespace of the nodes launcher
        """
        return cls.launcher.get_namespace()

    @classmethod
    def setup_class(cls):
        cls.launcher.startup()
        cls.launcher.launch_nodes(*cls.nodes)

    @classmethod
    def teardown_class(cls):
        cls.launcher.cleanup()


def is_debugger_attached(pid) -> bool:
    """
    Checks if a debugger is attached to a process

    Args:
        pid (int): The process id to check
    Returns:
        True if a debugger is attached, False otherwise
    """
    try:
        with open(f"/proc/{pid}/status") as file:
            for line in file:
                tracer_pid = line.find("TracerPid:")
                if tracer_pid >= 0:
                    return line[tracer_pid:].split()[1] != "0"
    except Exception:
        return False
    return False


def wait_for_debugger(pid_or_exec_name):
    """
    Waits for a debugger to attach to the process

    Args:
        pid_or_node (int or str): The pid of the process or the executable to wait for
            If a string is passed, the first process with that executable name will be waited for
    """
    if isinstance(pid_or_exec_name, str):
        pid = int(check_output(["pidof", "-s", pid_or_exec_name]).strip())
    else:
        pid = pid_or_exec_name

    while not is_debugger_attached(pid):
        print(f"Waiting for debugger to attach to {pid}...")
        time.sleep(0.5)


def publish_to_topic(topic_type: type, topic: str, msg):
    """
    Publishes `msg` to `topic` using the `topic_type` type

    Requires `msg` is of type `topic_type`
    """
    node = RclpyNode(f"cmr_py_test_client_{int(datetime.utcnow().timestamp() * 1000)}")
    client = node.create_publisher(topic_type, topic, 10)

    client.publish(msg)
    published = False

    def publish():
        nonlocal published
        client.publish(msg)
        node.destroy_node()
        published = True

    timer = node.create_timer(0.1, publish)
    while not published:
        ros.spin_once(node)


class TopicSubscriber:
    """
    A class for listening to a topic
    """

    def __init__(self, msg_type: type, topic: str):
        """
        Constructs a new TopicSubscriber that listens for messages of type `msg_type`
        on the topic `topic`
        """
        self.node = RclpyNode(
            f"cmr_py_topic_subscriber_{int(datetime.utcnow().timestamp() * 1000)}"
        )
        self.results = []
        self.destroyed = False
        self.sub = self.node.create_subscription(msg_type, topic, self._callback, 10)

    def _callback(self, msg):
        self.results.append(msg)

    def __del__(self):
        if not self.destroyed:
            self.node.destroy_node()
            self.destroyed = True

    def __exit__(self, exc_type, exc_value, traceback):
        if not self.destroyed:
            self.node.destroy_node()
            self.destroyed = True

    def __enter__(self):
        return self

    def wait_for_msg(self, timeout_sec=None, async_nodes=[]):
        """
        Waits for a message to be received on the topic and returns that message

        Args:
            timeout_sec (float or None): The number of seconds to wait for a message
                If None, waits forever
            async_nodes (list of RclpyNode): A list of nodes to spin while waiting
        Returns:
            The message received or None if the timeout is reached
        """
        start_time = datetime.now()
        while len(self.results) == 0 and (
            timeout_sec is None
            or (datetime.now() - start_time).total_seconds() < timeout_sec
        ):
            ros.spin_once(self.node)
            for node in async_nodes:
                ros.spin_once(node)
        return self.results.pop(0) if len(self.results) > 0 else None


class ServiceListener:
    """
    A class for listening to service calls
    """

    def __init__(self, srv_type: type, service: str, callback):
        """
        Constructs a new ServiceListener that listens for messages of type `srv_type`
        on the service `service`

        Args:
            srv_type (type): The type of the service
            service (str): The name of the service
            callback (function): The callback to call when a service request is received
                Accepts the request and response as arguments and should update the response
                parameter and return the response
        """
        self.node = RclpyNode(
            f"cmr_py_topic_subscriber_{int(datetime.utcnow().timestamp() * 1000)}"
        )
        self.request_count = 0
        self.destroyed = False
        self.sub = self.node.create_service(srv_type, service, self._callback)
        self.cb = callback

    def _callback(self, request, response):
        self.request_count += 1
        return self.cb(request, response)

    def __del__(self):
        if not self.destroyed:
            self.node.destroy_node()
            self.destroyed = True

    def __exit__(self, exc_type, exc_value, traceback):
        if not self.destroyed:
            self.node.destroy_node()
            self.destroyed = True

    def __enter__(self):
        return self

    def wait_for_msg(self, wait_pred=1, timeout_sec=None, async_nodes=[]):
        """
        Waits for a message to be received on the service

        Args:
            wait_pred (int or callable): The number of requests to wait for or a function
                that returns True when we can stop waiting
            timeout_sec (float or None): The number of seconds to wait for a message
                If None, waits forever
            async_nodes (list of RclpyNode): A list of nodes to spin while waiting
        Returns:
            True if the message was received, False if the timeout is reached
        """

        self.request_count = 0

        def can_stop_waiting():
            if isinstance(wait_pred, int):
                return self.request_count >= wait_pred
            elif callable(wait_pred):
                return wait_pred()

        start_time = datetime.now()
        while (not can_stop_waiting()) and (
            timeout_sec is None
            or (datetime.now() - start_time).total_seconds() < timeout_sec
        ):
            ros.spin_once(self.node)
            for node in async_nodes:
                ros.spin_once(node)
        return can_stop_waiting()
