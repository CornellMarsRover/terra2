#!/usr/bin/env python3

import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory


def main():
    package_share = get_package_share_directory('autonomous_navigation')
    tool_path = os.path.join(package_share, 'tooling', 'Live_telemetry_tool.py')

    if not os.path.exists(tool_path):
        raise FileNotFoundError(f'Live telemetry tool not found at {tool_path}')

    spec = importlib.util.spec_from_file_location('autonomous_navigation_live_telemetry_tool', tool_path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Unable to load telemetry tool module from {tool_path}')

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.main()

