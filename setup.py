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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lexmaister",
    maintainer_email="lexmaister@gmail.com",
    description="Simple ROS2 package for simulating thermophilic turtles moving in a temperature field",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"field_node = {package_name}.field_node:main"],
    },
)
