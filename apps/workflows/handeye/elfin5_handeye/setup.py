from glob import glob
from os.path import isfile
from setuptools import setup

package_name = 'elfin5_handeye'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[('share/' + package_name + '/' + folder,
           [path for path in glob(folder + '/*') if isfile(path)])
          for folder in ('launch', 'urdf', 'config', 'worlds')],
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Elfin5 station maintainers', maintainer_email='maintainer@example.com',
    description='Gazebo Harmonic eye-in-hand calibration', license='Apache-2.0',
    entry_points={'console_scripts': [
        'calibrate = elfin5_handeye.calibrate:main',
    ]},
)
