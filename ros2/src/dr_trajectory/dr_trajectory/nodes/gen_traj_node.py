import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dr_interfaces.srv import SolveIK
from dr_interfaces.srv import SolveFK
from dr_interfaces.srv import GenTraj
from dr_trajectory.trajectory_generator import TrajectoryGenerator
from dr_interfaces.msg import RobotState


class GenTrajNode(Node):
    
    def __init__(self):
        super().__init__('gen_traj_node')

        self.declare_parameter('q_max',         rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('q_min',         rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qd_max',          rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qdd_max',         rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('ef_v_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_a_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_omega_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_alpha_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ctrl_freq', rclpy.Parameter.Type.DOUBLE)

        params = {
            'q_max':            self.get_parameter('q_max').value,
            'q_min':            self.get_parameter('q_min').value,         
            'qd_max':           self.get_parameter('qd_max').value,
            'qdd_max':          self.get_parameter('qdd_max').value,
            'ef_v_max':         self.get_parameter('ef_v_max').value,
            'ef_a_max':         self.get_parameter('ef_a_max').value,
            'ef_omega_max':     self.get_parameter('ef_omega_max').value,
            'ef_alpha_max':     self.get_parameter('ef_alpha_max').value,
            'ctrl_freq':        self.get_parameter('ctrl_freq').value
        }

        self.declare_parameter('q0', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.cb_group = ReentrantCallbackGroup()

        self.robot_state_sub = self.create_subscription(RobotState, '/dr/robot_state', self.robot_state_callback, 10, callback_group=self.cb_group)

        self.ik_client = self.create_client(SolveIK, '/dr/solve_ik', callback_group=self.cb_group)
        self.fk_client = self.create_client(SolveFK, '/dr/solve_fk', callback_group=self.cb_group)

        self.q0 = list(self.get_parameter('q0').value)
        self.c0 = [0.0] * 6

        self.generator = TrajectoryGenerator(params, self.call_ik, self.call_fk, logger=self.get_logger())

        self.srv = self.create_service(GenTraj, '/dr/gen_traj',self.gen_traj_callback, callback_group=self.cb_group)

        self._init_timer = self.create_timer(0.1, self._init_c0, callback_group=self.cb_group)
        self.get_logger().info('GenTraj node iniciado.')

    def _init_c0(self):
        self._init_timer.cancel()
        self.fk_client.wait_for_service()
        _, self.c0 = self.call_fk(self.q0)
        self.get_logger().info(f'Estado inicial: q0 = {[f"{v:.3f}" for v in self.q0]}, c0 = {[f"{v:.3f}" for v in self.c0]}')

    def call_ik(self, pose, force=False):
        request = SolveIK.Request()
        request.pose = list(pose)
        request.q0 = list(self.q0)
        request.force = force
        future = self.ik_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()
        return future.result().success, list(future.result().q)

    def call_fk(self, q):
        request = SolveFK.Request()
        request.q = list(q)
        future = self.fk_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()
        return future.result().success, list(future.result().pose)
    
    def gen_traj_callback(self, request, response):
        modo_str  = {0: 'Joint', 1: 'Linear'}.get(request.mode, str(request.mode))
        speed_str = {1: 'Lento', 2: 'Medio',  3: 'Rápido'}.get(request.speed_profile, str(request.speed_profile))
        pose_str  = [f'{v:.3f}' for v in request.pose]
        self.get_logger().info(f'GenTraj solicitado → modo: {modo_str}, speed: {speed_str}, pose: {pose_str}')

        pose_obj = request.pose
        mode = request.mode
        speed = request.speed_profile

        [success, q_traj, points] = self.generator.generate(pose_obj, mode, speed, self.q0, self.c0)

        if success:
            response.success = True
            response.q_traj = q_traj
            response.points = points
            response.message = "Interpolación exitosa."
            self.get_logger().info(f'Trayectoria generada: {points} puntos.')
            self.q0 = list(q_traj[-6:])
            self.c0 = list(pose_obj)
        else:
            response.success = False
            response.q_traj = q_traj
            response.points = points
            response.message = "Interpolación fallida."
            self.get_logger().warn('GenTraj falló para la pose solicitada.')

        return response

    def robot_state_callback(self, msg):
        self.q0 = list(msg.q)
        self.c0 = list(msg.pose)

def main(args=None):
    rclpy.init(args=args)
    node = GenTrajNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()