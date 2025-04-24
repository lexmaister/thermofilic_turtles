from setuptools import find_packages, setup
from glob import glob

package_name = "thermofilic_turtles"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", glob("launch/*.launch.py")),
        (f"share/{package_name}/resource", glob("resource/*.bmp")),
    ],
    install_requires=[
        "setuptools",
        "ament_index_python",
    ],
    zip_safe=True,
    maintainer="lexmaister",
    maintainer_email="lexmaister@gmail.com",
    description="Simple ROS2 package for simulating thermophilic turtles moving in a temperature field",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"field_publisher = {package_name}.field_publisher_node:main",
            f"ctrl_ready_checker = {package_name}.ctrl_ready_node:main",
            f"kinesis_ctrl = {package_name}.kinesis_ctrl_node:main",
            f"turtle_spawner = {package_name}.turtle_spawner_node:main",
        ],
    },
)
