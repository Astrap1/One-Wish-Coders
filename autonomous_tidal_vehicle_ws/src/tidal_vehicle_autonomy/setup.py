from setuptools import find_packages, setup

package_name = "tidal_vehicle_autonomy"

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
    entry_points={
        "console_scripts": ["global_planner = tidal_vehicle_autonomy.global_planner_node:main"],
    },
    maintainer="One Wish Coders",
    maintainer_email="team@example.com",
    description="Perception, planning, avoidance and mission logic.",
    license="Apache-2.0",
)
