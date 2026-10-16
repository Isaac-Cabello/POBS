from setuptools import find_packages, setup

package_name = 'pobs_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pobs_gazebo']),
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
	('share/pobs_gazebo/worlds', ['worlds/empty.sdf']),
    ('share/pobs_gazebo/worlds', ['worlds/underwater.sdf']),
    ],
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
            'trajectory_comparison = '
            'pobs_gazebo.trajectory_comparison:main',
        ],
    },
)
