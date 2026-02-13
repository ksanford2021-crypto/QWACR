from setuptools import setup

package_name = "qwacr_comms"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/qwacr_comms"]),
        ("share/qwacr_comms", ["package.xml"]),
        ("share/qwacr_comms/launch", [
            "launch/robot_comms.launch.py",
            "launch/base_station.launch.py",
        ]),
        ("share/qwacr_comms/config", [
            "config/lora_config.yaml",
            "config/halow_config.yaml",
        ]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="kyle",
    maintainer_email="kyle@example.com",
    description="LoRa and HaLow communications bridge for QWACR.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lora_bridge = qwacr_comms.lora_bridge:main",
            "command_mux = qwacr_comms.command_mux:main",
            "telemetry_publisher = qwacr_comms.telemetry_publisher:main",
        ],
    },
)
