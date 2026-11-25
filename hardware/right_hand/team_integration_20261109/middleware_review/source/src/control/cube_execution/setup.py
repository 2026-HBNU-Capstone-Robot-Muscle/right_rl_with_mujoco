from setuptools import find_packages, setup

package_name = 'cube_execution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/hardware_calibration.example.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonjihwan',
    maintainer_email='jeonjihwan@todo.todo',
    description='Safe dry-run executor for cube solutions',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cube_sequence_executor = cube_execution.executor_node:main',
            'cube_calibration_check = cube_execution.calibration:main',
        ],
    },
)
