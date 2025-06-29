from setuptools import setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ika',
    maintainer_email='ika@example.com',
    description='Obstacle avoidance system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
            'fake_lidar_publisher = obstacle_avoidance.fake_lidar_publisher:main',
        ],
    },
)

