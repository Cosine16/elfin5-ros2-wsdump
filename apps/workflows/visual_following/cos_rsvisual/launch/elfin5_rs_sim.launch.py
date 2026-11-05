#!/usr/bin/python3

# elfin5_rs_sim.launch.py:
# Elfin5 + RealSense D435i (眼在手上) GAZEBO + MoveIt!2 + 视觉跟随 一键仿真。
#
# 以 elfin5_ros2_moveit2/launch/elfin5.launch.py 为蓝本, 差异:
#   - world 换成本包的 elfin5_actor.world (带行走假人)
#   - robot_description 用本包 elfin5_rs_d435i.urdf.xacro (含 D435i 相机)
#   - robot_description_semantic 用本包 elfin5_rs.srdf (含相机 disable_collisions)
#   - move_group 追加 sensors_3d.yaml (octomap 点云更新)
#   - 追加 elfin_basic_api_node (/cart_goal 自动规划执行)
#   - 追加 vision_container 组件容器: PersonDepthDetector(默认) + ArmFollower,
#     PersonVisualDetector 由 enable_visual_detector:=true 切换(与深度互斥)

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:

        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    # ***** LAUNCH ARGUMENTS ***** #
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    # 是否改用 HOG 视觉检测器(与深度检测器互斥)
    declare_enable_visual_detector_cmd = DeclareLaunchArgument(
        name='enable_visual_detector',
        default_value='False',
        description='Load PersonVisualDetector instead of PersonDepthDetector')
    # 眼在手上外参(真机手眼标定结果, 仿真用默认即可)
    declare_mount_xyz_cmd = DeclareLaunchArgument(
        name='mount_xyz',
        default_value='0 0 0.02',
        description='D435i camera mount position on elfin_end_link (hand-eye calibration)')
    declare_mount_rpy_cmd = DeclareLaunchArgument(
        name='mount_rpy',
        default_value='0 0 0',
        description='D435i camera mount orientation on elfin_end_link (hand-eye calibration)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_visual_detector = LaunchConfiguration('enable_visual_detector')

    # *********************** Gazebo *********************** #

    # Gazebo WORLD: 本包带行走假人的 world
    rs_world = os.path.join(
        get_package_share_directory('cos_rsvisual'),
        'worlds',
        'elfin5_actor.world')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': rs_world}.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # 本包的 elfin5 + D435i xacro (gazebo 仿真: 两个 hardware arg 都置 false)。
    # 用 Command 在运行时展开, 使 mount_xyz / mount_rpy launch 参数生效。
    xacro_file = os.path.join(
        get_package_share_directory('cos_rsvisual'),
        'urdf',
        'elfin5_rs_d435i.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=false',
         ' mount_xyz:="', LaunchConfiguration('mount_xyz'), '"',
         ' mount_rpy:="', LaunchConfiguration('mount_rpy'), '"'])
    # foxy 会对字符串参数尝试 yaml 解析, URDF 里的注释冒号会触发解析错误,
    # 必须用 ParameterValue(..., value_type=str) 显式声明为纯字符串
    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'elfin5', "-x", "0.0", "-y", "0.0", "-z", "0.1"],
                        output='screen')

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    # 注意: 整条仿真链路(含 rsp/move_group/rviz/basic_api)必须统一 use_sim_time,
    # 否则 gazebo 用仿真时间戳而其余节点用系统时间戳, TF 与图像时间戳对不上,
    # 检测器的 tf 变换会一直等待(已在 gdb 中实证阻塞在 canTransform)。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "elfin_base_link"],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ***** CONTROLLERS ***** #
    # ros2_control: controller_manager 由 gazebo_ros2_control 插件在 gzserver 内创建,
    # 不要启动独立的 ros2_control_node (会 SIGABRT, 详见 elfin5.launch.py 注释)。
    load_controllers = []
    for controller in [
        "elfin_arm_controller",
        "joint_state_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # *********************** MoveIt!2 *********************** #

    # Command-line argument: RVIZ file
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # Robot description, SRDF: 本包(含相机 disable_collisions)
    robot_description_semantic_config = load_file(
        "cos_rsvisual", "config/elfin5_rs.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "elfin5_ros2_moveit2", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "elfin5_ros2_moveit2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "elfin5_ros2_moveit2", "config/elfin_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # 注意: 不再向 move_group 注入 sensors_3d.yaml —— Foxy 的 MoveIt deb (2.2.3)
    # 没有编译 PointCloudOctomapUpdater 插件(foxy 分支 occupancy_map_monitor 的
    # CMakeLists 里就不含 updater), 注入只会报 plugin 加载失败。障碍物感知改由
    # obstacle_updater 组件把 /person_pose 以 collision object 形式写进规划场景。
    # sensors_3d.yaml 保留在 config/ 中, 供升级 Humble+ 后启用 octomap 方案。

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    # RVIZ: 本包配置(含 PointCloud2 + PersonPose display)
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_full_config = os.path.join(
        get_package_share_directory("cos_rsvisual"), "rviz", "elfin5_rs.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    # *********************** elfin_basic_api *********************** #
    # /cart_goal (PoseStamped, 参考系 world) 自动规划执行
    elfin_basic_api_node = Node(
        name="elfin_basic_node",
        package="elfin_basic_api",
        executable="elfin_basic_api_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic,
                    robot_description_kinematics,
                    {'use_sim_time': use_sim_time}],
    )

    # *********************** 视觉组件容器 *********************** #
    follower_yaml = os.path.join(
        get_package_share_directory("cos_rsvisual"), "config", "follower.yaml")

    # 注意: foxy 的 launch_ros 把传给 ComposableNode 的 yaml 文件整体扁平化成
    # "节点名.ros__parameters.参数名" 的参数列表发给容器, rclcpp 不做分节匹配,
    # 参数静默不生效(此前靠代码默认值恰好等于 yaml 才没暴露, 如 max_range 2.0/2.9)。
    # 这里自行读 yaml 取出对应节点分节, 以纯 dict 形式传入(按精确名生效)。
    def _load_section(section):
        with open(follower_yaml) as f:
            data = yaml.safe_load(f)
        return data[section]['ros__parameters']

    # foxy 的 ComposableNode 不支持 condition, 用 OpaqueFunction 在运行时
    # 根据 enable_visual_detector 组装组件列表(深度/视觉检测器互斥)
    def _make_vision_container(context):
        use_visual = LaunchConfiguration(
            'enable_visual_detector').perform(context).lower() in ('true', '1', 'yes')
        if use_visual:
            detector = ComposableNode(
                package='cos_rsvisual',
                plugin='cos_rsvisual::PersonVisualDetector',
                name='person_visual_detector',
                parameters=[_load_section('person_visual_detector'),
                            {'use_sim_time': use_sim_time}],
            )
        else:
            detector = ComposableNode(
                package='cos_rsvisual',
                plugin='cos_rsvisual::PersonDepthDetector',
                name='person_depth_detector',
                parameters=[_load_section('person_depth_detector'),
                            {'use_sim_time': use_sim_time}],
            )
        follower = ComposableNode(
            package='cos_rsvisual',
            plugin='cos_rsvisual::ArmFollower',
            name='arm_follower',
            # MoveGroupInterface 需要 robot_description + robot_description_semantic + kinematics
            parameters=[_load_section('arm_follower'),
                        {'use_sim_time': use_sim_time},
                        robot_description, robot_description_semantic,
                        robot_description_kinematics],
        )
        obstacle_updater = ComposableNode(
            package='cos_rsvisual',
            plugin='cos_rsvisual::ObstacleUpdater',
            name='obstacle_updater',
            parameters=[_load_section('obstacle_updater'),
                        {'use_sim_time': use_sim_time}],
        )
        return [ComposableNodeContainer(
            name='vision_container',
            namespace='',
            package='rclcpp_components',
            # 多线程容器: 检测回调耗 CPU, 单线程会把参数服务/TF/定时器饿死
            executable='component_container_mt',
            composable_node_descriptions=[detector, follower, obstacle_updater],
            output='screen',
        )]

    vision_container = OpaqueFunction(function=_make_vision_container)

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_enable_visual_detector_cmd,
            declare_mount_xyz_cmd,
            declare_mount_rpy_cmd,
            rviz_arg,

            # 禁用 FastDDS 共享内存传输(残留 shm 段会导致发现/订阅随机失效)
            SetEnvironmentVariable(
                'FASTRTPS_DEFAULT_PROFILES_FILE',
                os.path.join(get_package_share_directory('cos_rsvisual'),
                             'config', 'fastrtps_udp_only.xml')),

            # Gazebo nodes:
            gazebo,
            spawn_entity,

            # ROS2_CONTROL:
            # 注意: spawn_entity 以 volatile 订阅 /robot_description, 若 robot_state_publisher
            # 先发布(一次性)则 spawn 永远等不到(时序竞争, 已实测偶发)。故 rsp 延后 3s 启动,
            # 确保 spawn 先完成订阅。
            static_tf,
            TimerAction(period=3.0, actions=[robot_state_publisher]),

            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_entity,
                    on_exit = [

                        # MoveIt!2:
                        rviz_node_full,
                        run_move_group_node,

                        # 视觉检测 + 跟随:
                        vision_container,

                        # /cart_goal 规划执行(放在最后启动: 避免其 MoveGroupInterface
                        # 在 move_group 就绪前初始化导致后续 cart_goal 不触发规划)
                        TimerAction(period=8.0, actions=[elfin_basic_api_node]),

                    ]
                )
            )
        ]
        + load_controllers
    )
