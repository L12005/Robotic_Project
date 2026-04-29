from setuptools import find_packages, setup


package_name = "hmi_behavior"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="li",
    maintainer_email="li@todo.todo",
    description="Behavior controller skeleton for the HMI robot yielding demo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "behavior_node = hmi_behavior.behavior_node:main",
        ],
    },
)
