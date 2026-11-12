from setuptools import find_packages, setup

package_name = 'leap_hand_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/policy_np.npz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonjihwan',
    maintainer_email='jeonjihwan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leap_hand_primitive_server = leap_hand_control.primitive_server:main',
            'leap_policy_shadow = leap_hand_control.policy_shadow_node:main',
        ],
    },
)
