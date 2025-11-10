#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import uuid
import subprocess
import threading

from coresense_msgs.srv import StartSession, AddToSession, ListSession # , GetSolution
# from session_manager_pkg.action import SolveSession

class SessionManager(Node):
    def __init__(self):
        super().__init__('session_manager')

        self.sessions = {}      # session_id -> list of strings
        self.solutions = {}     # session_id -> string

        # Services
        self.start_srv = self.create_service(StartSession, 'start_session', self.start_session_cb)
        self.add_srv = self.create_service(AddToSession, 'add_to_session', self.add_to_session_cb)
        self.list_srv = self.create_service(ListSession, 'list_session', self.list_session_cb)
        # self.get_sol_srv = self.create_service(GetSolution, 'get_solution', self.get_solution_cb)

        # Action
        '''
        self.solve_action = ActionServer(
            self,
            SolveSession,
            'solve_session',
            execute_callback=self.execute_solve_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        '''

        self.get_logger().info("SessionManager node ready.")

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

    '''
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
        self.get_logger().info("Solver goal canceled.")
        return CancelResponse.ACCEPT

    async def execute_solve_cb(self, goal_handle):
        sid = goal_handle.request.session_id
        goal_handle.publish_feedback(SolveSession.Feedback(feedback=f"Launching solver for {sid}..."))

        data = "\n".join(self.sessions[sid])
        solver_proc = subprocess.Popen(
            ["ros2", "run", "solver_pkg", "solver_exec"],  # or path to your solver binary
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        def feed_solver():
            try:
                solver_proc.stdin.write(data)
                solver_proc.stdin.close()
            except Exception as e:
                self.get_logger().error(f"Error feeding solver: {e}")

        threading.Thread(target=feed_solver, daemon=True).start()
        out, err = solver_proc.communicate()

        if solver_proc.returncode != 0:
            self.get_logger().error(f"Solver failed: {err}")
            goal_handle.succeed()
            result = SolveSession.Result(success=False, result=err)
        else:
            self.get_logger().info(f"Solver succeeded for {sid}")
            self.solutions[sid] = out.strip()
            result = SolveSession.Result(success=True, result=out.strip())

        return result
    '''

def main(args=None):
    rclpy.init(args=args)
    node = SessionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
