from setuptools import find_packages, setup

package_name = 'covapsy_tools'

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
    maintainer='covapsy',
    maintainer_email='team@example.com',
    description='CoVAPSy baseline package: ' + package_name,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_logger_node = covapsy_tools.telemetry_logger_node:main',
            'param_sanity_checker = covapsy_tools.param_sanity_checker:main',
            'drive_cmd_to_twist_adapter = covapsy_tools.drive_cmd_to_twist_adapter:main',
        ]
    },
)
