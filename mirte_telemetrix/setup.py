from setuptools import setup
from glob import glob

package_name = "mirte_telemetrix"
submodules = "mirte_telemetrix/mappings"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Martin Klomp",
    maintainer_email="m.klomp@tudelft.nl",
    description="TODO: Package description",
    license="AGPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mirte_async = mirte_telemetrix.asynctest:main",
            "mirte_telemetrix = mirte_telemetrix.ROS_telemetrix_aio_api:start",
        ],
    },
)
