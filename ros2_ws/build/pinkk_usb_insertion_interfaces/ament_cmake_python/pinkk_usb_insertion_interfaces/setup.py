from setuptools import find_packages
from setuptools import setup

setup(
    name='pinkk_usb_insertion_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('pinkk_usb_insertion_interfaces', 'pinkk_usb_insertion_interfaces.*')),
)
