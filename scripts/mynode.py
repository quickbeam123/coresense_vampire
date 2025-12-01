#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import uuid
import subprocess
import time
import select

from coresense_msgs.srv import StartSession, AddToSession, ListSession, GetSolution, TptpExternal
from coresense_msgs.action import QueryReasoner

import os, socket
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
        return CancelResponse.ACCEPT

    def call_service(self, srv_type, srv_name, request,
                    availability_timeout=1.0,
                    response_timeout=2.0):

        # Create client
        client = self.create_client(srv_type, srv_name)

        # Check availability with timeout
        available = client.wait_for_service(timeout_sec=availability_timeout)
        if not available:
            raise RuntimeError(f"Service {srv_name} not available after {availability_timeout}s")

        # Call asynchronously
        future = client.call_async(request)

        # Wait for response (executor threads will handle callbacks)
        start = time.time()
        while not future.done():
            if time.time() - start > response_timeout:
                raise TimeoutError(f"Service {srv_name} did not respond within {response_timeout}s")
            time.sleep(0.01)   # yield without busy spinning

        return future.result()

    def execute_solve_cb(self, goal_handle):
        sid = goal_handle.request.session_id
        goal_handle.publish_feedback(QueryReasoner.Feedback(status=f"Launching solver for {sid}..."))

        prefix = get_package_prefix('coresense_vampire')
        exe = os.path.join(prefix, 'lib', 'coresense_vampire', 'vampire_z3_rel_static_martin-xdb-coresense_10526')

        parent_sock, child_sock = socket.socketpair(socket.AF_UNIX)
        sock_file = parent_sock.makefile("rwb", buffering=0)

        with parent_sock, sock_file:
            # self.get_logger().info(f"child_sock.fileno() was {child_sock.fileno()}")

            solver_proc = subprocess.Popen(
                # "valgrind --leak-check=full --track-origins=yes".split()+
                [exe,"-esfd",str(child_sock.fileno())]+goal_handle.request.configuration.split(),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=[child_sock.fileno()],   # keep this fd open across exec
                text=True
            )

            child_sock.close()

            data = "\n".join(self.sessions[sid]+[goal_handle.request.query])

            try:
                solver_proc.stdin.write(data)
                solver_proc.stdin.close()
            except Exception:
                pass

            # Monitor process in a loop
            while solver_proc.poll() is None:  # still running
                self.get_logger().info("Polling...")
                time.sleep(0.1)

                # Non-blocking readiness check
                readable, _, _ = select.select([parent_sock], [], [], 0)
                if readable:
                    line = sock_file.readline()
                    if line: # otherwise child closed FD ?
                        msg = line.decode().rstrip()
                        self.get_logger().info(f"Received: {msg}")

                        try:
                            service_name,q = msg.split()
                            answer = self.call_service(TptpExternal, service_name, TptpExternal.Request(question=q)).answer
                            self.get_logger().info(f"Sending back {len(answer)} answer lines:")
                            for a in answer:
                                self.get_logger().info(f"   {a}")
                                sock_file.write(f"{a}\n".encode())
                        except Exception as e:
                            self.get_logger().info(f"Exception during external service call: {e}")

                        # TODO: this write will fail, had Vampire timed-out in the meantime...
                        sock_file.write(f"\n".encode())
                        sock_file.flush()

                # Check for cancellation from client
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Cancel requested, stopping solver...")

                    solver_proc.terminate()

                    try:
                        solver_proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        solver_proc.kill()

                    goal_handle.canceled()
                    result = QueryReasoner.Result()
                    result.result = "Canceled"
                    result.code = 2 # arbitrarily (let's define this conventions properly later)
                    return result

            solver_proc.stdin = None # so that communicate won't touch stdin
            out, err = solver_proc.communicate()

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
    rclpy.spin(node,executor = rclpy.executors.MultiThreadedExecutor())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
