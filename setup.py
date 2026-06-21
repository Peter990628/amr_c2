from setuptools import find_packages, setup

package_name = 'mini_pjt'

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
    maintainer='rokey',
    maintainer_email='haebyuk35@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_start = mini_pjt.robot_start:main',
            'waypoint_node = mini_pjt.waypoint_node:main',
            'waypoint_node_2 = mini_pjt.waypoint_node_2:main',
            'yolo_publisher_amr = mini_pjt.yolo_publisher_amr:main',
            'yolo_subscriber_amr = mini_pjt.yolo_subscriber_amr:main',
            'yolov8_obj_det_amr = mini_pjt.yolov8_obj_det_amr:main',
        ],
    },
)
