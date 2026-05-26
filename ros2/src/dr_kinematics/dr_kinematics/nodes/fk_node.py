import rclpy
from rclpy.node import Node
from dr_interfaces.srv import SolveFK
from dr_kinematics.forw_kinematics import ForwKinematics


class FKNode(Node):

    def __init__(self):
        super().__init__("fk_node")

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

        self.solver = ForwKinematics(params)
        self.srv = self.create_service(SolveFK, "/dr/solve_fk", self.solve_fk_callback)
        self.get_logger().info("FK node iniciado.")

    def solve_fk_callback(self, request, response):

        q1 = request.q[0]
        q2 = request.q[1]
        q3 = request.q[2]
        q4 = request.q[3]
        q5 = request.q[4]
        q6 = request.q[5]

        [success, pose] = self.solver.solve_fk(q1, q2, q3, q4, q5, q6)

        if success:
            response.success = success
            response.pose = pose
            response.message = "Solución para FK encontrada"
        else:
            response.success = success
            response.pose = [0.0] * 6
            response.message = "Fallo al calcular FK"
            self.get_logger().warn(f"FK falló para q = {list(request.q)}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
