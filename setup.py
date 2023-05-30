from setuptools import setup

package_name = 'ros2_streamer'

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
    maintainer='JB Mouret',
    maintainer_email='jean-baptiste.mouret@inria.fr',
    description='A ROS2 "interface" to gst_launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = ros2_streamer.gstreamer_service:main',

        ],
    },
)
