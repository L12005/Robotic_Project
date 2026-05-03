from setuptools import find_packages, setup


package_name = 'hmi_test'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scene_topic_monitors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='li@todo.todo',
    description='Test nodes for validating HMI scene topic publishers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_monitor = hmi_test.robot_state_monitor:main',
            'human_state_monitor = hmi_test.human_state_monitor:main',
            'obstacle_state_monitor = hmi_test.obstacle_state_monitor:main',
            'map_state_monitor = hmi_test.map_state_monitor:main',
            'map_grid_dump = hmi_test.map_grid_dump:main',
        ],
    },
)
