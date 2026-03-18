from setuptools import setup

package_name = "stanford_arm_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/ros2_controllers.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bryan",
    maintainer_email="bryanlee6722@gmail.com",
    description="Stanford arm PID effort control",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_pid_node = stanford_arm_control.joint_pid_node:main",
        ],
    },
)