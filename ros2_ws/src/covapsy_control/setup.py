from setuptools import find_packages, setup

package_name = 'covapsy_control'

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
            'pure_pursuit_node = covapsy_control.pure_pursuit_node:main',
            'mode_controller_node = covapsy_control.mode_controller_node:main',
        ]
    },
)
