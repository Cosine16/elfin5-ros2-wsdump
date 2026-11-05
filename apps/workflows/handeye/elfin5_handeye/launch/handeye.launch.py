"""Humble + Harmonic simulation: one robot_description and one TF authority."""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('elfin5_handeye'))
    moveit = Path(get_package_share_directory('elfin5_ros2_moveit2'))
    description = xacro.process_file(str(share / 'urdf/elfin5_d435i.urdf.xacro'), mappings={
        key: LaunchConfiguration(key).perform(context) for key in ('mount_xyz', 'mount_rpy')
    }).toxml()
    srdf = ET.parse(moveit / 'config/elfin5.srdf').getroot()
    for a, b in [('elfin_link6', 'camera_bracket'), ('elfin_end_link', 'camera_bracket'),
                 ('camera_bracket', 'camera_link')]:
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Adjacent')
    parameters = {
        'robot_description': description,
        'robot_description_semantic': ET.tostring(srdf, encoding='unicode'),
        'robot_description_kinematics': yaml.safe_load((moveit / 'config/kinematics.yaml').read_text()),
        'use_sim_time': True,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join('default_planner_request_adapters/' + name for name in (
                'AddTimeOptimalParameterization', 'FixWorkspaceBounds', 'FixStartStateBounds',
                'FixStartStateCollision', 'FixStartStatePathConstraints')),
            'start_state_max_bounds_error': 0.1,
            **yaml.safe_load((moveit / 'config/ompl_planning.yaml').read_text()),
        },
        'robot_description_planning': {'joint_limits': {
            f'elfin_joint{i}': {'has_velocity_limits': True, 'max_velocity': 1.0,
                              'has_acceleration_limits': True, 'max_acceleration': 1.0}
            for i in range(1, 7)}},
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['elfin_arm_controller'],
            'elfin_arm_controller': {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': [f'elfin_joint{i}' for i in range(1, 7)]}},
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 3.0,
        'trajectory_execution.allowed_start_tolerance': 0.02,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
    }
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
        get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': ('-r -s --headless-rendering ' if headless else '-r ') +
                          str(share / 'worlds/handeye.sdf')}.items())
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'handeye', '-topic', 'robot_description', '-name', 'elfin5'])
    js = Node(package='controller_manager', executable='spawner', output='screen',
              arguments=['joint_state_broadcaster', '--controller-manager-timeout', '90'])
    arm = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['elfin_arm_controller', '--controller-manager-timeout', '90'])

    def after_success(next_actions):
        def callback(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Spawn/controller startup failed'))]
            return next_actions
        return callback

    return [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success([js]))),
        RegisterEventHandler(OnProcessExit(target_action=js, on_exit=after_success([arm]))),
        gz,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen'),
        spawn,
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[parameters], output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[parameters],
             arguments=['-d', str(moveit / 'launch/elfin5_moveit2.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.07'),
        DeclareLaunchArgument('mount_rpy', default_value='0 -1.5707963267948966 0'),
        OpaqueFunction(function=setup),
    ])
