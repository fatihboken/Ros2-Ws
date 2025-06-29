from setuptools import find_packages, setup

package_name = 'ika_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='ika',
    maintainer_email='ika@todo.todo',
    description='Dummy camera publisher and subscriber example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'dummy_camera_node = ika_robot.dummy_camera_node:main',
        'dummy_camera_subscriber = ika_robot.dummy_camera_subscriber:main',
    ],
},

)

