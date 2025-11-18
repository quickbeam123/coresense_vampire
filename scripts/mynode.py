#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import uuid
import subprocess
import threading

from coresense_msgs.srv import StartSession, AddToSession, ListSession, GetSolution
from coresense_msgs.action import QueryReasoner

import os
from ament_index_python.packages import get_package_prefix

class VampireRunner(Node):
    def __init__(self):
        super().__init__('session_manager')

        self.sessions = {}      # session_id -> list of strings
        self.solutions = {}     # session_id -> string

        # Services
        self.start_srv = self.create_service(StartSession, 'start_session', self.start_session_cb)
        self.add_srv = self.create_service(AddToSession, 'add_to_session', self.add_to_session_cb)
        self.list_srv = self.create_service(ListSession, 'list_session', self.list_session_cb)
        self.get_sol_srv = self.create_service(GetSolution, 'get_solution', self.get_solution_cb)

        # Action
        self.solve_action = ActionServer(
            self,
            QueryReasoner,
            'query_reasoner',
            execute_callback=self.execute_solve_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info("VampireRunner node ready.")

    # ---- Services ----
    def start_session_cb(self, request, response):
        session_id = str(uuid.uuid4())
        self.sessions[session_id] = []
        self.get_logger().info(f"Created session {session_id}")
        response.session_id = session_id
        return response

    def add_to_session_cb(self, request, response):
        sid = request.session_id
        if sid not in self.sessions:
            response.success = False
            return response
        self.sessions[sid].append(request.formula)
        response.success = True
        return response

    def list_session_cb(self, request, response):
        sid = request.session_id
        if sid not in self.sessions:
            response.success = False
            response.formulas = []
            return response
        response.success = True
        response.formulas = self.sessions[sid]
        return response

    def get_solution_cb(self, request, response):
        sid = request.session_id
        if sid not in self.solutions:
            response.success = False
            response.solution = ""
        else:
            response.success = True
            response.solution = self.solutions[sid]
        return response

    # ---- Actions ----
    def goal_cb(self, goal_request):
        self.get_logger().info(f"Received goal for session {goal_request.session_id}")
        if goal_request.session_id not in self.sessions:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info(f"Request to cancel solve action for session {goal_handle.request.session_id}")
        # TODO: have a means of killing a running Vampire
        return CancelResponse.REJECT

    async def execute_solve_cb(self, goal_handle):
        sid = goal_handle.request.session_id
        goal_handle.publish_feedback(QueryReasoner.Feedback(status=f"Launching solver for {sid}..."))

        prefix = get_package_prefix('coresense_vampire')
        exe = os.path.join(prefix, 'lib', 'coresense_vampire', 'vampire_z3_rel_static_master_10435')

        solver_proc = subprocess.Popen(
            [exe]+goal_handle.request.configuration.split(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        data = "\n".join(self.sessions[sid]+[goal_handle.request.query])
        out, err = solver_proc.communicate(input=data)

        result = QueryReasoner.Result()
        result.code = solver_proc.returncode
        result.result = f"Out:\n{out}\nErr:\n{err}"

        if solver_proc.returncode != 0:
            self.get_logger().error(f"Solver failed:\nOut:\n{out}\nErr:\n{err}")
            goal_handle.abort()
        else:
            self.get_logger().info(f"Solver succeeded for {sid}")
            self.solutions[sid] = out.strip()
            goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    node = VampireRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
