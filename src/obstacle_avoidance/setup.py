from setuptools import setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fatih',
    maintainer_email='fatih@example.com',
    description='Engelden kaçan robot için ROS 2 paketi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_lidar_publisher = obstacle_avoidance.fake_lidar_publisher:main',
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main'
        ],
    },
)

