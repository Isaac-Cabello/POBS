<<<<<<< HEAD
from pathlib import Path
=======
import os
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47

from setuptools import find_packages, setup

package_name = 'pobs_gazebo'
<<<<<<< HEAD
model_files = []
for model in ('pobs', 'pobs_cad', 'lake_terrain', 'sonar_targets', '2-wheeled_bot'):
    for directory in (Path('models') / model).rglob('*'):
        if directory.is_file() and directory.suffix not in ('.bak',):
            model_files.append((
                'share/' + package_name + '/' + str(directory.parent),
                [str(directory)],
            ))
=======
cad_mesh = 'models/pobs_cad/meshes/Chassis.STL'
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pobs_gazebo']),
<<<<<<< HEAD
        ('share/pobs_gazebo', ['package.xml', 'TERRAIN.md']),
        ('share/pobs_gazebo/launch', ['launch/simulation.launch.py',
                                    'launch/trajectory_comparison.launch.py']),
        ('share/pobs_gazebo/rviz', ['rviz/trajectory_comparison.rviz']),
        ('share/pobs_gazebo/worlds', ['worlds/empty.sdf', 'worlds/lake.sdf',
                                    'worlds/lake_cad.sdf']),
    ] + model_files,
=======
        ('share/pobs_gazebo', ['package.xml', 'VISUALIZATION.md']),
        ('share/pobs_gazebo/launch', [
            'launch/simulation.launch.py',
            'launch/trajectory_comparison.launch.py',
        ]),
        ('share/pobs_gazebo/rviz', ['rviz/trajectory_comparison.rviz']),
        ('share/pobs_gazebo/models/lake_terrain',
            ['models/lake_terrain/model.sdf', 'models/lake_terrain/model.config']),
        ('share/pobs_gazebo/models/lake_terrain/meshes', [
            'models/lake_terrain/meshes/terrain.obj',
            'models/lake_terrain/meshes/water.obj',
        ]),
        ('share/pobs_gazebo/models/pobs',
            ['models/pobs/model.sdf', 'models/pobs/model.config']),
        ('share/pobs_gazebo/models/pobs_cad',
            ['models/pobs_cad/model.sdf', 'models/pobs_cad/model.config']),
        ('share/pobs_gazebo/worlds',
            ['worlds/empty.sdf', 'worlds/lake.sdf', 'worlds/lake_cad.sdf']),
    ] + ([('share/pobs_gazebo/models/pobs_cad/meshes', [cad_mesh])]
         if os.path.isfile(os.path.join(os.path.dirname(__file__), cad_mesh))
         else []),
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47
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
<<<<<<< HEAD
            'trajectory_comparison = pobs_gazebo.trajectory_comparison:main',
=======
            'trajectory_comparison = '
            'pobs_gazebo.trajectory_comparison:main',
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47
        ],
    },
)
