from setuptools import setup
import os
from glob import glob

package_name = 'ros2_nav_efrei'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name,'map'), glob('map/*')),
        (os.path.join('share', package_name,'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name,'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlebot',
    maintainer_email='turtlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
        'console_scripts': [
            'n2pac = ros2_nav_efrei.action_server_client:main',
        ],
    },
)

