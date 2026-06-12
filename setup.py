from setuptools import setup

package_name = "eimdall_ros2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yves BIGLIAZZI",
    maintainer_email="yves.bigliazzi@data-agility.fr",
    description="ROS 2 bridge for Eimdall edge runtime.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "health_bridge = eimdall_ros2_bridge.health_bridge:main",
            "anomaly_bridge = eimdall_ros2_bridge.anomaly_bridge:main",
            "ingest_bridge = eimdall_ros2_bridge.ingest_bridge:main",
        ],
    },
)
