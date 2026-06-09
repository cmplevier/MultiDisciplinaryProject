from setuptools import find_packages, setup

package_name = 'mdp_state'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ignacio Quintanilla Rojas',
    maintainer_email='iquintanillaro@tudelft.nl',
    description='State node for MDP 2026 — persists scan results to SQLite',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'state_node = mdp_state.state_node:main',
        ],
    },
)
