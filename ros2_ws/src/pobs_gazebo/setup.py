import os

from setuptools import find_packages, setup

package_name = 'pobs_gazebo'
cad_mesh = 'models/pobs_cad/meshes/Chassis.STL'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pobs_gazebo']),
        ('share/pobs_gazebo', ['package.xml']),
        ('share/pobs_gazebo/launch', ['launch/simulation.launch.py']),
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
        ],
    },
)
