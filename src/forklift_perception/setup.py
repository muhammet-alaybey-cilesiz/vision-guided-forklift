from glob import glob

from setuptools import find_packages, setup

PKG = 'forklift_perception'

setup(
    name=PKG,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG]),
        ('share/' + PKG, ['package.xml']),
        ('share/' + PKG + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammet61',
    maintainer_email='muhammetclsz61@gmail.com',
    description='Forklift camera perception (ROS-free core + ROS nodes)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
