#!/usr/bin/python3

# elfin5_rs_face_real.launch.py:
# 真机人脸跟随: RealSense D435i(USB) + Haar 人脸检测 + ArmFollower 末端相机对准人脸。
#
# 前置(本 launch 不包含, 需先启动):
#   机械臂硬件 + MoveIt 栈, 用 cos_ws/scripts/start_real.py 按顺序拉起
#   (EtherCAT ros2_control_node 需要 capsh 用户态授权, 见该脚本说明),
#   并在 Elfin Control Panel 里 Clear Fault -> Servo On。
#   硬件栈的 robot_state_publisher 发布 world -> elfin_end_link 的 TF,
#   move_group 接收本包 face_follower 的规划请求。
#
# 本 launch 启动:
#   1. realsense2_camera: 彩色 640x480@15 + 深度对齐到彩色(align_depth),
#      发布相机自身 TF (camera_link -> *_optical_frame);
#   2. static_transform_publisher: elfin_end_link -> camera_link
#      (眼在手上外参 mount_xyz/mount_rpy, 手眼标定后改这两个 launch 参数),
#      把相机 TF 树挂到机械臂 TF 树上;
#   3. vision_container 组件容器:
#      - FaceVisualDetector: 彩色图 Haar 人脸检测 + 对齐深度取距 -> /face_pose
#      - ArmFollower(节点名 face_follower): 订阅 /face_pose,
#        内置 MoveGroupInterface 规划执行, 末端位置不变、转动姿态让相机对准人脸。
#
# 运行时开关:
#   ros2 param set /face_visual_detector enable false
#   ros2 param set /face_follower enable_follow false
#
# USB 带宽注意: 彩色 + 对齐深度在 USB2.1 下建议 640x480@15(默认值);
# 若 rs-enumerate-devices 显示协商到 USB3, 可提高到 640x480x30。

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


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
    # 眼在手上外参(真机手眼标定结果, 初装后可用默认值先用着再标定)
    declare_mount_xyz_cmd = DeclareLaunchArgument(
        name='mount_xyz',
        default_value='0 0 0.02',
        description='D435i camera mount position on elfin_end_link (hand-eye calibration)')
    declare_mount_rpy_cmd = DeclareLaunchArgument(
        name='mount_rpy',
        default_value='0 0 0',
        description='D435i camera mount orientation on elfin_end_link (hand-eye calibration)')
    declare_enable_follow_cmd = DeclareLaunchArgument(
        name='enable_follow',
        default_value='True',
        description='ArmFollower 跟随总开关(也可运行时 ros2 param set 切换)')

    # *********************** RealSense *********************** #
    # 4.51+ (ros2 分支) 参数名; 若用 3.2.3 旧版驱动, 对应参数见
    # rs_camera.launch.py 头部注释(depth_width/enable_color 等)。
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'depth_module.profile': '640x480x15',
            'rgb_module.profile': '640x480x15',
            'enable_color': 'true',
            # 深度对齐到彩色: 人脸框小, 非对齐深度的像素错位会毁掉框内取距
            'align_depth.enable': 'true',
            # 跟随不依赖点云, 关掉省 USB 带宽
            'pointcloud.enable': 'false',
        }.items(),
    )

    # *********************** 视觉组件容器 *********************** #
    face_yaml = os.path.join(
        get_package_share_directory('cos_rsvisual'), 'config', 'face_follow.yaml')

    # robot_description(含相机 link)注入跟随器:
    # 其内置 MoveGroupInterface 需要它构造 RobotModel(规划组/关节名)。
    # 注意与硬件栈 elfin5_moveit.launch.py 的 URDF 不同源: 那份不含相机。
    # 本 launch 不再起 robot_state_publisher(硬件栈已发布关节 TF),
    # 相机 TF 由 static_transform_publisher + realsense 驱动补足。
    xacro_file = os.path.join(
        get_package_share_directory('cos_rsvisual'),
        'urdf',
        'elfin5_rs_d435i.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=true',
         ' mount_xyz:="', LaunchConfiguration('mount_xyz'), '"',
         ' mount_rpy:="', LaunchConfiguration('mount_rpy'), '"'])
    # foxy 会对字符串参数尝试 yaml 解析, 必须显式声明为纯字符串
    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    # MoveGroupInterface 还需要 SRDF(规划组 elfin_arm 定义)和运动学配置,
    # 缺了会报 "Group 'elfin_arm' was not found"(实测)
    robot_description_semantic = {'robot_description_semantic': load_file(
        'cos_rsvisual', 'config/elfin5_rs.srdf')}
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(
        'elfin5_ros2_moveit2', 'config/kinematics.yaml')}

    # mount_xyz/mount_rpy 是 "x y z" / "r p y" 字符串, static_transform_publisher
    # 需要拆成独立参数, 用 OpaqueFunction 在运行时解析组装
    #
    # 注意: foxy 的 launch_ros 把传给 ComposableNode 的 yaml 文件整体扁平化成
    # "节点名.ros__parameters.参数名" 的参数列表发给容器, rclcpp 不做分节匹配,
    # 参数静默不生效(follower.yaml 在仿真里就是靠默认值恰好等于 yaml 才没暴露)。
    # 因此这里自行读 yaml 取出对应节点分节, 以纯 dict 形式传入(按精确名生效)。
    def _load_section(section):
        with open(face_yaml) as f:
            data = yaml.safe_load(f)
        return data[section]['ros__parameters']

    def _make_camera_tf_and_container(context):
        xyz = LaunchConfiguration('mount_xyz').perform(context).split()
        rpy = LaunchConfiguration('mount_rpy').perform(context).split()
        camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_tf',
            output='log',
            arguments=xyz + rpy + ['elfin_end_link', 'camera_link'],
            parameters=[{'use_sim_time': False}],
        )
        detector = ComposableNode(
            package='cos_rsvisual',
            plugin='cos_rsvisual::FaceVisualDetector',
            name='face_visual_detector',
            parameters=[_load_section('face_visual_detector'),
                        {'use_sim_time': False}],
        )
        follower = ComposableNode(
            package='cos_rsvisual',
            plugin='cos_rsvisual::ArmFollower',
            name='face_follower',
            parameters=[_load_section('face_follower'),
                        {'use_sim_time': False,
                         'enable_follow': LaunchConfiguration('enable_follow')},
                        robot_description, robot_description_semantic,
                        robot_description_kinematics],
        )
        container = ComposableNodeContainer(
            name='vision_container',
            namespace='',
            package='rclcpp_components',
            # 多线程容器: Haar 检测在订阅回调里耗 CPU, 单线程容器会把
            # 参数服务/TF 监听/跟随定时器全部饿死(实测 param 服务超时)
            executable='component_container_mt',
            composable_node_descriptions=[detector, follower],
            output='screen',
        )
        return [camera_tf, container]

    camera_tf_and_container = OpaqueFunction(function=_make_camera_tf_and_container)

    return LaunchDescription([
        declare_mount_xyz_cmd,
        declare_mount_rpy_cmd,
        declare_enable_follow_cmd,

        # 禁用 FastDDS 共享内存传输(残留 shm 段会导致发现/订阅随机失效);
        # 建议同时 export ROS_LOCALHOST_ONLY=1(与 start_real.py 一致)
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(get_package_share_directory('cos_rsvisual'),
                         'config', 'fastrtps_udp_only.xml')),

        rs_launch,
        camera_tf_and_container,
    ])
