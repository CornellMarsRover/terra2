---
name: terra-cpp
description: Create or modify Terra2 C++17 ROS code. Use for rclcpp nodes, lifecycle/fabric code, CMake targets, headers, libraries, and C++ tests that must satisfy this repository's clang-format and clang-tidy configuration.
---
# Terra2 C++ code

Read the package `CMakeLists.txt`, `package.xml`, public headers, and neighboring
tests first. Prefer the repository's `cmr_add_lib`, `cmr_add_node`,
`cmr_add_test`, `cmr_install`, and `cmr_export` helpers where the package already
uses them. Add dependencies to both CMake and `package.xml`.

The checked-in `.clang-format` is authoritative: C++17, four spaces, no tabs,
85-column limit, Linux braces, sorted includes, `CamelCase` classes/structs,
lower-case functions/variables, and `m_` private/protected members. Do not hand
format around the tool.

Keep ROS transport at node boundaries and put reusable behavior in libraries.
Make ownership and lifetime explicit, avoid blocking executor callbacks, and
preserve lifecycle/fault behavior. Add C++ tests through the package's existing
CMake helper and cover errors and boundary values.

Build before static analysis so `build/compile_commands.json` exists. Run
`clang-format` on changed files or pre-commit, then
`bash scripts/check_wd.sh CHANGED_FILES`. Treat clang-tidy readability,
bugprone, and diagnostic findings as errors because CI does.
