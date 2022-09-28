from test_utils import *


@cmr_node_test
def test_demo_node(node_launcher: NodeLauncher):
    node_name = "py_test_demo"
    node_launcher.launch_nodes(
        make_fabric_node(
            config_string=f"""
        package = "cmr_demo"
        executable = "demo_node"
        name = "{node_name}"
        dependencies = []

        [fault_handling]
        restart_attempts = 2
        restart_delay = 10

        [node]
        test="Hi"
        """
        )
    )
    assert get_lifecycle_state(node_name) == "unconfigured"
    activate_fabric_node(node_name, node_launcher.get_namespace())
    assert get_lifecycle_state(node_name) == "active"
