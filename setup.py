from setuptools import setup

package_name = 'ros2_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('lib/' + package_name, [package_name+'/v4l2.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JB Mouret',
    maintainer_email='jean-baptiste.mouret@inria.fr',
    description='A ROS2 "interface" to gst_launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = ros2_streamer.gstreamer_service:main',
            'watchdog = ros2_streamer.watchdog:main',
            'test_sub = ros2_streamer.test_sub:main',

        ],
    },
)
