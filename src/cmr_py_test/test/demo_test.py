from test_utils import *

static_config = """
package = "cmr_demo"
executable = "demo_node"
name = "py_test_demo"
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 10

[node]
test="Hi"
"""

# Test functions should be decorated with @cmr_node_test
# and should accept the test namespace as an argument
@cmr_node_test([make_fabric_node(config_string=static_config)])
def test_demo_node(namespace: str):
    assert get_lifecycle_state("py_test_demo") == "unconfigured"
    activate_fabric_node("py_test_demo", namespace)
    assert get_lifecycle_state("py_test_demo") == "active"


node_config = """
    package = "cmr_demo"
    executable = "demo_node"
    name = "{}"
    dependencies = [{}]

    [fault_handling]
    restart_attempts = 2
    restart_delay = 2

    [node]
    test="Hi"
"""


# Test fixtures must start with `Test` and test methods must start with `test_`
# Use Test fixtures when you want to run different tests with the same nodes
class TestDemoDependencies(CMRTestFixture):
    CMRTestFixture.nodes = config_fabric_nodes(
        node_config.format("py_test_demo_1", ""),
        node_config.format("py_test_demo_2", '"py_test_demo_1"'),
    )

    def test_demo_activation(self):
        node_name = "py_test_demo_2"
        activate_fabric_node(node_name, CMRTestFixture.get_namespace())
        assert get_lifecycle_state(node_name) == "active"
        assert get_lifecycle_state("py_test_demo_1") == "active"

    def test_depender_deactivate(self):
        activate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
        deactivate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
        assert get_lifecycle_state("py_test_demo_2") == "inactive"
        assert get_lifecycle_state("py_test_demo_1") == "inactive"

    def test_dependee_deactivate(self):
        activate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
        deactivate_fabric_node("py_test_demo_1", CMRTestFixture.get_namespace())
        assert get_lifecycle_state("py_test_demo_2") == "inactive"
        assert get_lifecycle_state("py_test_demo_1") == "inactive"
