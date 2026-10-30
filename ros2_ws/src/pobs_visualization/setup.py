from setuptools import find_packages, setup


setup(
    name='pobs_visualization',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pobs_visualization']),
        ('share/pobs_visualization', ['package.xml', 'README.md']),
        ('share/pobs_visualization/launch', ['launch/visualization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='POBS team',
    maintainer_email='isaac.cabello76@gmail.com',
    description='Selectable RViz displays for POBS state estimation and sonar mapping.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'trajectory_comparison = pobs_visualization.trajectory_comparison:main',
        'clock_bridge = pobs_visualization.clock_bridge:main',
        'lake_reference_scene = pobs_visualization.scene:main',
    ]},
)
