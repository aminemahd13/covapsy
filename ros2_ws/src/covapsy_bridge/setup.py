from setuptools import find_packages, setup

package_name = 'covapsy_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='covapsy',
    maintainer_email='team@example.com',
    description='CoVAPSy baseline package: ' + package_name,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = covapsy_bridge.stm32_bridge_node:main',
            'dynamixel_steering_node = covapsy_bridge.dynamixel_steering_node:main',
        ]
    },
)
