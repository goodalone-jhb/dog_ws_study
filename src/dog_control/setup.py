from setuptools import find_packages, setup

package_name = 'dog_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cquick',
    maintainer_email='cquick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'joint_command_publisher = dog_control.joint_command_publisher:main',
        'joint_command_bridge = dog_control.joint_command_bridge:main',
        'stand_controller = dog_control.stand_controller:main',
        'damiao_driver = dog_control.damiao_driver:main',
        ],
    },
)
