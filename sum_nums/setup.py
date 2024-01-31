from setuptools import setup

package_name = 'sum_nums'

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
    description='ROS2 pub-sub nodes to find sum of n positive integers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sum_nums_pub = sum_nums.sum_nums_publisher:main',
            'sum_nums_sub = sum_nums.sum_nums_subscriber:main',
            'sum_nums_server = sum_nums.sum_nums_service_server:main',
            'sum_nums_client = sum_nums.sum_nums_service_client:main',
        ],
    },
)
