from pathlib import Path

from setuptools import find_packages, setup

package_name = 'pobs_gazebo'
model_files = []
for model in ('pobs', 'pobs_cad', 'lake_terrain', 'sonar_targets', '2-wheeled_bot'):
    for directory in (Path('models') / model).rglob('*'):
        if directory.is_file() and directory.suffix not in ('.bak',):
            model_files.append((
                'share/' + package_name + '/' + str(directory.parent),
                [str(directory)],
            ))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pobs_gazebo']),
        ('share/pobs_gazebo', ['package.xml', 'TERRAIN.md']),
        ('share/pobs_gazebo/launch', ['launch/simulation.launch.py',
                                    'launch/trajectory_comparison.launch.py']),
        ('share/pobs_gazebo/rviz', ['rviz/trajectory_comparison.rviz']),
        ('share/pobs_gazebo/worlds', ['worlds/empty.sdf', 'worlds/lake.sdf',
                                    'worlds/lake_cad.sdf']),
    ] + model_files,
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac-Cabello',
    maintainer_email='isaac.cabello76@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_comparison = pobs_gazebo.trajectory_comparison:main',
        ],
    },
)
