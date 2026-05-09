from glob import glob
from setuptools import find_packages, setup


package_name = 'hmi_feedback'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='li@todo.todo',
    description='Visual feedback controllers for the HMI robot yielding demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_strip_controller = hmi_feedback.led_strip_controller:main',
        ],
    },
)
