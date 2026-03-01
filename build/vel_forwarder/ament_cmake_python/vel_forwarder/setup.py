from setuptools import find_packages
from setuptools import setup

setup(
    name='vel_forwarder',
    version='0.0.0',
    packages=find_packages(
        include=('vel_forwarder', 'vel_forwarder.*')),
)
