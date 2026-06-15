from setuptools import setup

package_name = "robot_arm_opencr_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/opencr_moveit.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuhei",
    maintainer_email="yuhei@example.com",
    description="FollowJointTrajectory to OpenCR serial bridge for the custom robot arm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "opencr_trajectory_bridge = robot_arm_opencr_bridge.opencr_trajectory_bridge:main",
        ],
    },
)
