from setuptools import find_packages, setup

package_name = 'pobs_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pobs_gazebo']),
        ('share/pobs_gazebo', ['package.xml']),
	('share/pobs_gazebo/launch', ['launch/simulation.launch.py']),
	('share/pobs_gazebo/worlds', ['worlds/empty.sdf']),
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
        ],
    },
)
