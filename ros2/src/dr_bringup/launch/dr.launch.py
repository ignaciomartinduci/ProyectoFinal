from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os



def generate_launch_description():

    params_path = os.path.join(
        get_package_share_directory("dr_interfaces"), "config", "robot_params.yml"
    )

    with open(params_path, "r") as f:
        raw = yaml.safe_load(f)

    robot_params = raw["robot"]["ros__parameters"]

    return LaunchDescription(
        [
            LogInfo(msg=""),
            LogInfo(msg="==============================="),
            LogInfo(msg="Universidad Nacional de Cuyo"),
            LogInfo(msg="Facultad de Ingeniería"),
            LogInfo(msg="Ignacio Martín Duci - Legajo: 13560"),
            LogInfo(msg="Año 2026"),
            LogInfo(msg="==============================="),
            LogInfo(msg=""),
            Node(
                package="dr_kinematics",
                executable="ik_node",
                name="ik_node",
                parameters=[robot_params],
            ),
            Node(
                package="dr_kinematics",
                executable="fk_node",
                name="fk_node",
                parameters=[robot_params],
            ),
            Node(
                package="dr_trajectory",
                executable="gen_traj_node",
                name="gen_traj_node",
                parameters=[robot_params],
            ),
            Node(
                package="dr_state",
                executable="state_node",
                name="state_node",
                parameters=[robot_params],
            ),
            Node(
                package="dr_executor",
                executable="executor_node",
                name="executor_node",
                parameters=[robot_params],
            ),
            Node(package="dr_storage", executable="storage_node", name="storage_node"),
            Node(
                package="dr_teaching", executable="teaching_node", name="teaching_node"
            ),
            Node(
                package="dr_gui",
                executable="gui_node",
                name="gui_node",
                parameters=[robot_params],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": open(
                            os.path.join(
                                get_package_share_directory("robot_description"),
                                "urdf",
                                "robot.urdf",
                            )
                        ).read()
                    }
                ],
            ),
            Node(
                package="dr_state",
                executable="joint_state_bridge_node",
                name="joint_state_bridge_node",
            ),
        ]
    )
