from setuptools import find_packages, setup

package_name = 'covapsy_perception'

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
    description='Perception pipeline: LiDAR filter + camera border detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_filter_node = covapsy_perception.scan_filter_node:main',
            'border_detect_node = covapsy_perception.border_detect_node:main',
            'opponent_detect_node = covapsy_perception.opponent_detect_node:main',
            'depth_obstacle_node = covapsy_perception.depth_obstacle_node:main',
        ],
    },
)
