#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import your generated service type
# Make sure the package name matches your workspace!
from coresense_msgs.srv import VampireExternalPredicate


class TptpExternalService(Node):

    def __init__(self):
        super().__init__('tptp_external_service')

        # Create the service
        self.srv = self.create_service(
            VampireExternalPredicate,
            'external',
            self.handle_tptp_query
        )

        self.get_logger().info("TPTP External service ready.")

    def handle_tptp_query(self, request, response):
        self.get_logger().info(f"Received question: {request.parameters}")

        response.answers = [f"c{i}" if p == "" else p for i,p in enumerate(request.parameters)]
        # response.answers = []

        self.get_logger().info(f"Returning answers: {response.answers}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TptpExternalService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
