from setuptools import setup

package_name = 'depth_to_laserscan'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/depth_to_laserscan.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nithish Kumar',
    maintainer_email='nith600e@gmail.com',
    description='Converts RealSense depth image to LaserScan for use with SLAM Toolbox',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_to_laserscan_node = depth_to_laserscan.depth_to_laserscan_node:main',
        ],
    },
)
