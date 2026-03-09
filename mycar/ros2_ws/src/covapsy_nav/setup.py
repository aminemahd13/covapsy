from setuptools import find_packages, setup

package_name = 'covapsy_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COVAPSY Team',
    maintainer_email='covapsy@team.com',
    description='Navigation: gap follower, pure pursuit, mode controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follower_node = covapsy_nav.gap_follower_node:main',
            'pure_pursuit_node = covapsy_nav.pure_pursuit_node:main',
            'mode_controller_node = covapsy_nav.mode_controller_node:main',
            'tactical_race_node = covapsy_nav.tactical_race_node:main',
            'racing_path_publisher_node = covapsy_nav.racing_path_publisher_node:main',
        ],
    },
)
