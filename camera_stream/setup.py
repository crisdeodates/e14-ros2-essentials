from setuptools import setup

package_name = 'camera_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS2 nodes to display the camera stream',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream_pub = camera_stream.camera_stream_publisher:main',
            'camera_stream_sub = camera_stream.camera_stream_subscriber:main',
        ],
    },
)
