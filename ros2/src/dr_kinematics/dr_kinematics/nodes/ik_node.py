import rclpy
from rclpy.node import Node
from dr_interfaces.srv import SolveIK
from dr_kinematics.inv_kinematics import InvKinematics


class IKNode(Node):

    def __init__(self):
        super().__init__("ik_node")

        # Declaración de parámetros

        # Arrays DH - sin default, tipo explícito
        self.declare_parameter("dh_theta_offset", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_d", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_a", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_alpha", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("q_min", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("q_max", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("qd_max", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("qdd_max", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("T_base.xyz", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("T_base.rpy", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("T_tool.xyz", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("T_tool.rpy", rclpy.Parameter.Type.DOUBLE_ARRAY)

        # Escalares - con default está bien
        self.declare_parameter("ef_v_max", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("ef_a_max", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("ef_omega_max", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("ef_alpha_max", rclpy.Parameter.Type.DOUBLE)

        # Construir diccionario de parámetros
        params = {
            "q_min": self.get_parameter("q_min").value,
            "q_max": self.get_parameter("q_max").value,
            "qd_max": self.get_parameter("qd_max").value,
            "qdd_max": self.get_parameter("qdd_max").value,
            "dh_theta_offset": self.get_parameter("dh_theta_offset").value,
            "dh_d": self.get_parameter("dh_d").value,
            "dh_a": self.get_parameter("dh_a").value,
            "dh_alpha": self.get_parameter("dh_alpha").value,
            "ef_v_max": self.get_parameter("ef_v_max").value,
            "ef_a_max": self.get_parameter("ef_a_max").value,
            "ef_omega_max": self.get_parameter("ef_omega_max").value,
            "ef_alpha_max": self.get_parameter("ef_alpha_max").value,
            "T_base_xyz": self.get_parameter("T_base.xyz").value,
            "T_base_rpy": self.get_parameter("T_base.rpy").value,
            "T_tool_xyz": self.get_parameter("T_tool.xyz").value,
            "T_tool_rpy": self.get_parameter("T_tool.rpy").value,
        }

        self.IK_solver = InvKinematics(params)
        self.srv = self.create_service(SolveIK, "/dr/solve_ik", self.solve_ik_callback)
        self.get_logger().info("IK node iniciado.")

    def solve_ik_callback(self, request, response):

        x = request.pose[0]
        y = request.pose[1]
        z = request.pose[2]
        roll = request.pose[3]
        pitch = request.pose[4]
        yaw = request.pose[5]
        force = request.force
        q0 = request.q0

        [success, q_sol] = self.IK_solver.solve_ik(x, y, z, roll, pitch, yaw, force, q0)

        if success:
            response.success = True
            response.q = q_sol
            response.message = "solución para IK encontrada"
        else:
            response.success = False
            response.q = q0
            response.message = "solución para IK no encontrada - retornando q0"
            self.get_logger().warn(
                f"IK falló para pose = [{x:.3f}, {y:.3f}, {z:.3f}, {roll:.3f}, {pitch:.3f}, {yaw:.3f}]"
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
