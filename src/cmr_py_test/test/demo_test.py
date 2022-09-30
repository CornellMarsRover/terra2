from test_utils import *
from std_msgs.msg._string import String
from cmr_msgs.action._test_target_position import TestTargetPosition as TargetPosition
import threading

# static_config = """
# package = "cmr_demo"
# executable = "demo_node"
# name = "py_test_demo"
# dependencies = []

# [fault_handling]
# restart_attempts = 2
# restart_delay = 10

# [node]
# test="Hi"
# """

# # Test functions should be decorated with @cmr_node_test
# # and should accept the test namespace as an argument
# @cmr_node_test([make_fabric_node(config_string=static_config)])
# def test_demo_node(namespace: str):
#     assert get_lifecycle_state("py_test_demo") == "unconfigured"
#     activate_fabric_node("py_test_demo", namespace)
#     assert get_lifecycle_state("py_test_demo") == "active"


# @cmr_node_test([make_node("utils_test_node", "cmr_py_test", "utils_test_node", "test")])
# def test_utils_node(namespace: str):
#     subber = TopicSubscriber(String, "/test/utils_test_node/test_out")
#     publish_to_topic(String, "/test/utils_test_node/test_in", String(data="Hi"))
#     assert subber.wait_for_msg().data == "Hi"

#     # Test action server
#     send_goal = TargetPosition.Goal()
#     send_goal.x = 1.0
#     send_goal.y = 2.0
#     send_goal.z = 3.0
#     success = False

#     def check_goal_handle(goal_handle: action.server.ServerGoalHandle):
#         nonlocal success
#         success = goal_handle.request == send_goal
#         goal_handle.succeed()
#         return success

#     action_server = ActionListener(
#         TargetPosition,
#         "/test/utils_test_node/test_return",
#         lambda handle: TargetPosition.Result(success=check_goal_handle(handle)),
#     )
#     t = threading.Thread(
#         target=send_action_goal_sync,
#         args=(TargetPosition, "/test/utils_test_node/test_action", send_goal),
#     )
#     t.start()
#     assert action_server.wait_for_msg()
#     assert success
#     t.join()


# node_config = """
#     package = "cmr_demo"
#     executable = "demo_node"
#     name = "{}"
#     dependencies = [{}]

#     [fault_handling]
#     restart_attempts = 2
#     restart_delay = 2

#     [node]
#     test="Hi"
# """


# # Test fixtures must start with `Test` and test methods must start with `test_`
# # Use Test fixtures when you want to run different tests with the same nodes
# # Test fixtures are more efficient than using cmr_node_test multiple times because
# # the setup and cleanup code is nontrivial
# class TestDemoDependencies(CMRTestFixture):
#     CMRTestFixture.nodes = config_fabric_nodes(
#         node_config.format("py_test_demo_1", ""),
#         node_config.format("py_test_demo_2", '"py_test_demo_1"'),
#     )

#     def test_demo_activation(self):
#         node_name = "py_test_demo_2"
#         activate_fabric_node(node_name, CMRTestFixture.get_namespace())
#         assert get_lifecycle_state(node_name) == "active"
#         assert get_lifecycle_state("py_test_demo_1") == "active"

#     def test_depender_deactivate(self):
#         activate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
#         deactivate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
#         assert get_lifecycle_state("py_test_demo_2") == "inactive"
#         assert get_lifecycle_state("py_test_demo_1") == "inactive"

#     def test_dependee_deactivate(self):
#         activate_fabric_node("py_test_demo_2", CMRTestFixture.get_namespace())
#         deactivate_fabric_node("py_test_demo_1", CMRTestFixture.get_namespace())
#         assert get_lifecycle_state("py_test_demo_2") == "inactive"
#        assert get_lifecycle_state("py_test_demo_1") == "inactive"

def test_terminate():
    assert True
