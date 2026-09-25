from setuptools import find_packages, setup

package_name = "tidal_vehicle_operator"

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
    maintainer="One Wish Coders",
    maintainer_email="team@example.com",
    description="Operator interface, visualisation and telemetry tooling.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "operator_dashboard = tidal_vehicle_operator.operator_dashboard_node:main",
            "browser_dashboard = tidal_vehicle_operator.web_dashboard_node:main",
        ],
    },
)
