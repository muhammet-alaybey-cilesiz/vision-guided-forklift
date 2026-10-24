from glob import glob

from setuptools import find_packages, setup

PKG = 'forklift_autonomy'

setup(
    name=PKG,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG]),
        ('share/' + PKG, ['package.xml']),
        ('share/' + PKG + '/config', glob('config/*.yaml')),
        ('share/' + PKG + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammet61',
    maintainer_email='muhammetclsz61@gmail.com',
    description='Forklift mission manager (mode, state machine, motion primitives)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['mission_manager = forklift_autonomy.node:main']},
)
