from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'odrive_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools',
                      "pyserial==3.5",
                      'transforms3d==0.4.1'
                    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = odrive_ros2_pkg.odrive_node:main'
        ],
    },
)
