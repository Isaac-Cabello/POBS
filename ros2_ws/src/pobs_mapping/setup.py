from setuptools import find_packages, setup


package_name = 'pobs_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='POBS team',
    maintainer_email='isaac.cabello76@gmail.com',
    description='Accumulated sonar point clouds from Gazebo ground truth.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'point_cloud_mapper = pobs_mapping.point_cloud_mapper:main',
    ]},
)
