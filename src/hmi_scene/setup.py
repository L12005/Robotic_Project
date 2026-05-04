from glob import glob
from setuptools import find_packages, setup


package_name = 'hmi_scene'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, glob('*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='li@todo.todo',
    description='Scene-side state publishing logic for the HMI demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actor_forward_test_controller = hmi_scene.actor_forward_test_controller:main',
            'human_motion_controller = hmi_scene.actor_forward_test_controller:main',
            'goal_pose_publisher = hmi_scene.goal_pose_publisher:main',
            'robot_cmd_vel_controller = hmi_scene.robot_cmd_vel_controller:main',
            'scene_state_publisher = hmi_scene.scene_state_publisher:main',
        ],
    },
)
