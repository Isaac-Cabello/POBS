from setuptools import find_packages, setup

package_name = 'pobs_master_planner'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/planner.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/master_planner.launch.py',
            'launch/planner_gazebo_dry_run.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='POBS team',
    maintainer_email='pobs@example.invalid',
    description='Supervisory XY waypoint planner for POBS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'master_planner = pobs_master_planner.master_planner:main',
            'gazebo_pose_to_odom = pobs_master_planner.gazebo_pose_to_odom:main',
            'xy_controller = pobs_master_planner.xy_controller:main',
            'sim_thruster_safety = pobs_master_planner.sim_thruster_safety:main',
        ],
    },
)
