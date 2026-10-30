from setuptools import find_packages, setup


package_name = 'pobs_sonar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/sonars.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/sonars.launch.py', 'launch/sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='POBS team',
    maintainer_email='isaac.cabello76@gmail.com',
    description='UART drivers for the POBS SEN0599 mapping sonars.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sen0599_driver = pobs_sonar.sen0599_driver:main',
            'simulated_sonars = pobs_sonar.simulated_sonars:main',
        ],
    },
)
