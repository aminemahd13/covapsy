from setuptools import find_packages, setup

package_name = 'covapsy_bridge'

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
    description='STM32 HAT communication bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = covapsy_bridge.stm32_bridge_node:main',
            'tft_status_node = covapsy_bridge.tft_status_node:main',
        ],
    },
)
