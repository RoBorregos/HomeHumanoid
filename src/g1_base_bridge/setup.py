from setuptools import find_packages, setup

package_name = "g1_base_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoBorregos",
    maintainer_email="a00841255@tec.mx",
    description=(
        "Bridges cmd_vel to the G1's locomotion backend "
        "(Gazebo VelocityControl / unitree_rl_lab)"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_base_bridge = g1_base_bridge.base_bridge_node:main",
        ],
    },
)
