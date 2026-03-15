# cmr_fabric

@brief Framework for node configuration, management, and fault handling.

## Fabric Nodes

A `cmr::fabric::FabricNode` is a base class for all nodes using the fabric framework.
These nodes are subtypes of [ros2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html).
They do however have extra configuration and fault handling properties.

Fabric Nodes must run while the `cmr::fabric::FaultHandler` and `cmr::fabric::DependencyHandler`
are also running nodes. The `LifecycleManager` may also be spawned as a node to provide
a [service interface](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html) 
for the activation and deactivation.

A fabric node can be launched by a Python launch file which reads a TOML config file.
The TOML config file should be passed to the `config_path` ROS parameter of the node.
A toml file should contain something like the following:

```
package = "cmr_arm"
executable = "inverse_kinematics"
name = "ik_config"
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 10

[node]
# Node specific configuration goes here
```

Then we can launch the node from Python with something like the following:

```Python
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cmr_fabric",
                executable="fault_handler",
                namespace=composition_ns,
            ),
            Node(
                package="cmr_fabric",
                executable="lifecycle_manager",
                namespace=composition_ns,
            ),
            fabric_node("/cmr/terra2/src/cmr_arm/config/ik_config.toml"),
        ]
    )


def fabric_node(conf_path: str) -> Node:
    result = load(conf_path) # Load TOML file into dictionary
    pkg = result["package"]
    executable = result["executable"]
    name = result["name"]

    return Node(
        package=pkg,
        executable=executable,
        name=name,
        exec_name=name,
        parameters=[
            {
                "config_path": conf_path,
                "composition_ns": composition_ns,
            }
        ],
    )
```

## Fabric Utilities

There are some utilities for interacting with Fabric Nodes within this package.
`create_config()`, `create_test_thread()` and other functions in `test_utils.hpp`
are here to help write C++ unit tests that utilizie Fabric Nodes.

The @ref LifecycleHelpers provide an easy to use C++ interface to control the activation,
deactivation, and lifecycle of nodes.