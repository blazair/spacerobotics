from setuptools import find_packages
from setuptools import setup

setup(
    name='control',
    version='0.0.1',
    packages=find_packages(
        include=('control', 'control.*')),
)
