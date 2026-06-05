from glob import glob

from setuptools import find_packages, setup

package_name = 'mdp_mainloop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dextertje',
    maintainer_email='d.h.exterkate@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mainloop_node = mdp_mainloop.mainloop_node:main',
            'high_level_planner_node = '
            'mdp_mainloop.high_level_planner_node:main',
            'row_plan_builder_node = '
            'mdp_mainloop.row_plan_builder_node:main',
            'row_plan_validator_node = '
            'mdp_mainloop.row_plan_validator_node:main',
            'auto_tray_waypoint_node = '
            'mdp_mainloop.auto_tray_waypoint_node:main',
        ],
    },
)
