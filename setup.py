from setuptools import find_packages, setup

package_name = "eimdall_ros2_bridge"

setup(
    name=package_name,
    version="0.9.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py"]),
        (f"share/{package_name}/config", ["config/bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DATA AGILITY",
    maintainer_email="contact@data-agility.fr",
    description="ROS 2 bridge for the Eimdall Edge runtime",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"bridge = {package_name}.bridge_node:main",
        ],
    },
)
