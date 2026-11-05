from setuptools import setup

package_name = 'cos_realsense'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/capture_heightmap.launch.py',
            'launch/real_pointcloud.launch.py',
            'launch/sim_pointcloud.launch.py',
            'launch/view_heightmap.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/pointcloud.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/d435i.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/demo.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fit',
    maintainer_email='fit@todo.todo',
    description='D435i point cloud demo for RViz and Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_projector_node = cos_realsense.depth_projector_node:main',
            'hand_eye_calib_node = cos_realsense.hand_eye_calib_node:main',
            'heightmap_builder_node = cos_realsense.heightmap_builder_node:main',
            'pointcloud_stats_node = cos_realsense.pointcloud_stats_node:main',
        ],
    },
)
