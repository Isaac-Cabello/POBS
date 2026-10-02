from setuptools import find_packages, setup

package_name = 'twowheel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'imu_filter = twowheel_control.imu_filter:main',
            'imu_drive = twowheel_control.imu_drive:main',
            'dead_reckoning = twowheel_control.dead_reckoning:main',
            'drive_xm = twowheel_control.drive_xm:main',
        ],
    },
)
