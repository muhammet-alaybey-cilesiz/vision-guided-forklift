from setuptools import find_packages, setup

PKG = 'forklift_evaluation'

setup(
    name=PKG,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG]),
        ('share/' + PKG, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammet61',
    maintainer_email='muhammetclsz61@gmail.com',
    description='Simulation-only evaluation (ground truth, overlays, acceptance tests)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'f1_probe = forklift_evaluation.f1_probe:main',
        'calibrate_cameras = forklift_evaluation.calibrate_cameras:main',
        'color_stats = forklift_evaluation.color_stats:main',
        'cv_eval = forklift_evaluation.cv_eval:main',
        'perf_probe = forklift_evaluation.perf_probe:main',
        'mission_acceptance = forklift_evaluation.mission_acceptance:main',
        'fault_acceptance = forklift_evaluation.fault_acceptance:main',
    ]},
)
