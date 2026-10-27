#!/usr/bin/env python3
"""
Simple ROS2 script to list services for specific nodes.
"""

import rclpy
from rclpy.node import Node
from pathlib import Path
import time
import sys


def _normalize_prefixes(names):
    prefixes = []
    for name in names:
        if not name.startswith('/'):
            name = '/' + name
        prefixes.append(name)
    return prefixes


def _is_default_param_service(service_name):
    default_param_services = {
        'describe_parameters',
        'get_parameters',
        'get_parameter_types',
        'list_parameters',
        'set_parameters',
        'set_parameters_atomically',
    }
    last_segment = service_name.rsplit('/', 1)[-1]
    return last_segment in default_param_services


def _collect_services_with_retry(node, retries=5, delay_sec=0.3):
    """Wait for graph discovery to stabilize and return services."""
    last_services = []
    for _ in range(retries):
        rclpy.spin_once(node, timeout_sec=delay_sec)
        services = node.get_service_names_and_types()
        if services and services == last_services:
            return services
        last_services = services
        time.sleep(delay_sec)
    return last_services

def main(args=None):
    rclpy.init(args=args)
    node = Node('service_lister')
    
    try:
        show_types = False
        output_path = Path('service_list.txt')
        target_names = {'pm_skills',  'pm_robot_primitive_skills', 'pm_moveit_server', 'assembly_manager', 'pm_vision_manager'}
        target_prefixes = _normalize_prefixes(target_names)

        lines = []
        lines.append("=" * 80)
        lines.append("Services for target nodes")
        lines.append("=" * 80)
        lines.append("")

        total_services = 0
        services = _collect_services_with_retry(node)
        grouped = {prefix: [] for prefix in target_prefixes}

        for service_name, types in services:
            if _is_default_param_service(service_name):
                continue
            for prefix in target_prefixes:
                if service_name.startswith(prefix + '/'):
                    grouped[prefix].append((service_name, types))
                    break

        for prefix in sorted(grouped.keys()):
            filtered = grouped[prefix]
            lines.append(f"Node: {prefix}")
            lines.append("-" * 80)

            if not filtered:
                lines.append("  (No non-default services found)")
                lines.append("  Count: 0")
                lines.append("")
                continue

            for service_name, types in sorted(filtered):
                short_name = service_name[len(prefix) + 1:]
                lines.append(f"  {short_name}")
                if show_types:
                    type_str = ', '.join(types) if isinstance(types, list) else types
                    lines.append(f"    Type: {type_str}")
                total_services += 1

            lines.append(f"  Count: {len(filtered)}")
            lines.append("")

        lines.append("=" * 80)
        lines.append(f"Total: {total_services} services")
        lines.append("=" * 80)
        lines.append("")

        output_text = "\n".join(lines)
        existing_text = ""
        if output_path.exists():
            existing_text = output_path.read_text(encoding='utf-8')

        if existing_text != output_text:
            output_path.write_text(output_text, encoding='utf-8')
            print(output_text)
            print(f"Updated list at: {output_path.resolve()}")
        else:
            print(output_text)
            print(f"List already up to date: {output_path.resolve()}")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
