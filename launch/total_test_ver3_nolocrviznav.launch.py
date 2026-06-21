# total_test_ver3_nolocrviznav.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 네임스페이스 정의
    ns = 'robot2'
    

    # ----------------------------------------------------------------------
    # 2. 내가 개발한 mini_pjt 패키지의 노드들 정의
    # ----------------------------------------------------------------------
    
    waypoint_nav_goal_node = Node(package='mini_pjt', executable='waypoint_nav_goal', namespace=ns, output='screen')
    yolo_pub_node = Node(package='mini_pjt', executable='yolo_publisher_amr', namespace=ns, output='screen')
    # yolo_sub_node = Node(package='mini_pjt', executable='yolo_subscriber_amr', namespace=ns, output='screen')
    yolo_det_node = Node(package='mini_pjt', executable='yolov8_obj_det_amr', namespace=ns, output='screen')
    coordinate_node = Node(package='mini_pjt', executable='coordinate', namespace=ns, output='screen')

    # cam_to_map 노드는 특수하게 TF 리맵핑 적용
    cam_to_map_node = Node(
        package='mini_pjt',
        executable='cam_to_map',
        output='screen',
        remappings=[
            ('/tf', f'/{ns}/tf'),
            ('/tf_static', f'/{ns}/tf_static')
        ]
    )


    delay_my_project_nodes = TimerAction(
        period=5.0,  
        actions=[
            waypoint_nav_goal_node,
            yolo_pub_node,
            # yolo_sub_node,
            yolo_det_node,
            coordinate_node,
            cam_to_map_node,
        ]
    )

    # ----------------------------------------------------------------------
    # 4. 최종 반환
    # ----------------------------------------------------------------------
    return LaunchDescription([
        delay_my_project_nodes  
    ])
