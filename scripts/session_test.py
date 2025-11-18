#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from coresense_msgs.srv import StartSession, AddToSession, ListSession, GetSolution

from rclpy.action import ActionClient
from coresense_msgs.action import QueryReasoner

VAMP_CODES = [
  "VAMP_RESULT_STATUS_SUCCESS",
  "VAMP_RESULT_STATUS_UNKNOWN",
  "VAMP_RESULT_STATUS_OTHER_SIGNAL",
  "VAMP_RESULT_STATUS_INTERRUPTED",
  "VAMP_RESULT_STATUS_UNHANDLED_EXCEPTION"]


class SessionTester(Node):
    def __init__(self):
        super().__init__('session_tester')

    def call_service(self, srv_type, srv_name, request):
        client = self.create_client(srv_type, srv_name)
        self.get_logger().info(f"Waiting for {srv_name}...")
        client.wait_for_service()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_reasoner_query(self, session_id, query, configuration=""):
        """
        Call the QueryReasoner action and wait for the result.
        """

        action_client = ActionClient(self, QueryReasoner, '/query_reasoner')

        self.get_logger().info("Waiting for QueryReasoner action server...")
        action_client.wait_for_server()

        goal_msg = QueryReasoner.Goal()
        goal_msg.session_id = session_id
        goal_msg.query = query
        goal_msg.configuration = configuration

        # Send the goal
        self.get_logger().info(f"Sending query: {query} under configuration: {configuration}")
        goal_future = action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Reasoner rejected the goal.")
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"Reasoner result: {result.result}")
        self.get_logger().info(f"Reasoner code: {result.code} (VAMP_CODE={VAMP_CODES[result.code]})")

        return result

    def run_get_solution(self, session_id):
        solution_req = GetSolution.Request()
        solution_req.session_id = session_id
        solution_res = self.call_service(GetSolution, '/get_solution', solution_req)

        if solution_res.success:
            self.get_logger().info(f"Get solution for session {session_id} successful and returned:\n{solution_res.solution}")
        else:
            self.get_logger().info(f"Get solution for session {session_id} failed")

    def run(self):
        # Start session
        start_req = StartSession.Request()
        start_res = self.call_service(StartSession, '/start_session', start_req)

        session_id = start_res.session_id
        self.get_logger().info(f"Session ID: {session_id}")

        # Add formulas
        for formula in [
            "include('SET010+0.ax').",
        ]:
            add_req = AddToSession.Request()
            add_req.session_id = session_id
            add_req.formula = formula

            add_res = self.call_service(AddToSession, '/add_to_session', add_req)
            self.get_logger().info(f"Added formula '{formula}': {add_res.success}")

        # List session
        list_req = ListSession.Request()
        list_req.session_id = session_id
        list_res = self.call_service(ListSession, '/list_session', list_req)
        self.get_logger().info(f"Session contents: {list_res.formulas}")

        self.run_get_solution(session_id)

        query = "cnf(b,negated_conjecture,a!=b)."
        reasoner_result = self.run_reasoner_query(session_id, query, configuration="--input_syntax tptp -t 1")
        # if reasoner_result:
        #    self.get_logger().info(f"The whole reasoner_result: {reasoner_result}")

        self.run_get_solution(session_id)


def main():
    rclpy.init()
    node = SessionTester()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()