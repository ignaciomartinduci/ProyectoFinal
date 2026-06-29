# Copyright 2026 Ignacio Martín Duci
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import (LogInfo, DeclareLaunchArgument, OpaqueFunction,
                             SetEnvironmentVariable, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():

    debug_arg         = DeclareLaunchArgument('debug',         default_value='false')
    debug_verbose_arg = DeclareLaunchArgument('debug_verbose', default_value='false')

    def on_debug(context):
        actions = []
        debug   = LaunchConfiguration('debug').perform(context).lower()         == 'true'
        verbose = LaunchConfiguration('debug_verbose').perform(context).lower() == 'true'
        if debug or verbose:
            actions += [
                SetEnvironmentVariable('DR_DEBUG', '1'),
                LogInfo(msg='\033[92m===== MODO DEBUG ON =====\033[0m'),
            ]
        if verbose:
            actions += [
                SetEnvironmentVariable('DR_DEBUG_VERBOSE', '1'),
                LogInfo(msg='\033[92m===== MODO VERBOSE ON ===\033[0m'),
            ]
        return actions

    def on_fail(name):
        def handler(event, _context):
            if event.returncode > 0:
                return [LogInfo(msg=f'\033[91mNODO {name} NO SE LANZÓ CORRECTAMENTE\033[0m')]
            return []
        return handler

    params_path = os.path.join(
        get_package_share_directory('dr_interfaces'),
        'config',
        'robot_params.yml'
    )

    with open(params_path, 'r') as f:
        raw = yaml.safe_load(f)

    robot_params = raw['robot']['ros__parameters']

    ik_node = Node(
        package='dr_kinematics',
        executable='ik_node',
        name='ik_node',
        parameters=[robot_params]
    )

    fk_node = Node(
        package='dr_kinematics',
        executable='fk_node',
        name='fk_node',
        parameters=[robot_params]
    )

    gen_traj_node = Node(
        package='dr_trajectory',
        executable='gen_traj_node',
        name='gen_traj_node',
        parameters=[robot_params]
    )

    state_node = Node(
        package='dr_state',
        executable='state_node',
        name='state_node',
        parameters=[robot_params]
    )

    executor_node = Node(
        package='dr_executor',
        executable='executor_node',
        name='executor_node',
        parameters=[robot_params]
    )

    storage_node = Node(
        package='dr_storage',
        executable='storage_node',
        name='storage_node'
    )

    teaching_node = Node(
        package='dr_teaching',
        executable='teaching_node',
        name='teaching_node'
    )

    gui_node = Node(
        package='dr_gui',
        executable='gui_node',
        name='gui_node',
        parameters=[robot_params]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('robot_description'),
                    'urdf', 'robot.urdf'
                )
            ).read()
        }]
    )

    joint_state_bridge_node = Node(
        package='dr_state',
        executable='joint_state_bridge_node',
        name='joint_state_bridge_node'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('dr_bringup'),
            'rviz', 'robot.rviz'
        )],
        output='screen'
    )

    nodes = [
        ('ik_node',                ik_node),
        ('fk_node',                fk_node),
        ('gen_traj_node',          gen_traj_node),
        ('state_node',             state_node),
        ('executor_node',          executor_node),
        ('storage_node',           storage_node),
        ('teaching_node',          teaching_node),
        ('gui_node',               gui_node),
        ('robot_state_publisher',  robot_state_publisher),
        ('joint_state_bridge_node',joint_state_bridge_node),
        ('rviz2',                  rviz2),
    ]

    return LaunchDescription([

        debug_arg,
        debug_verbose_arg,
        OpaqueFunction(function=on_debug),

        LogInfo(msg=''),
        LogInfo(msg='==============================='),
        LogInfo(msg='Universidad Nacional de Cuyo'),
        LogInfo(msg='Facultad de Ingeniería'),
        LogInfo(msg='Ignacio Martín Duci - Legajo: 13560'),
        LogInfo(msg='Año 2026'),
        LogInfo(msg='==============================='),
        LogInfo(msg=''),

        *[action for _, action in nodes],
        *[
            RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_fail(name)))
            for name, action in nodes
        ],

    ])
