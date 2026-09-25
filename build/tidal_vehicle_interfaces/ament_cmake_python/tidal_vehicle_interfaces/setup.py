from setuptools import find_packages
from setuptools import setup

setup(
    name='tidal_vehicle_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('tidal_vehicle_interfaces', 'tidal_vehicle_interfaces.*')),
)
