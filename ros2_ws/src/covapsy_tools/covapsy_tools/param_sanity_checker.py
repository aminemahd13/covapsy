#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class ParamSanityChecker(Node):
    def __init__(self) -> None:
        super().__init__('param_sanity_checker')
        checks = [
            ('/mode_controller_node', 'learn_speed_cap_mps'),
            ('/mode_controller_node', 'race_speed_cap_mps'),
            ('/reactive_driver_node', 'max_speed_mps'),
            ('/pure_pursuit_node', 'speed_max_mps'),
        ]
        for node_name, key in checks:
            future = self.get_parameters_client(node_name).get_parameters([key])
            self.get_logger().info(f'queried {node_name}.{key}: {future}')


def main(args=None):
    rclpy.init(args=args)
    node = ParamSanityChecker()
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
