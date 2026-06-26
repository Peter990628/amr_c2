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
    # 1. 터틀봇4 표준 오픈 소스 패키지들의 런칭 파일 포함 (Localization, Viz, Nav2)
    # ----------------------------------------------------------------------
    
    # A. Localization 실행
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'map': '/home/rokey/rokey_ws/maps/my_map_ver1.yaml'
        }.items()
    )

    # B. RViz2 시각화 실행 (turtlebot4_viz)
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_viz'), 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'namespace': f'/{ns}'}.items()
    )

    # C. Nav2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'namespace': f'/{ns}'}.items()
    )

    # ----------------------------------------------------------------------
    # 2. 내가 개발한 mini_pjt 패키지의 노드들 정의
    # ----------------------------------------------------------------------
    
    waypoint_node = Node(package='mini_pjt', executable='waypoint_node_2', namespace=ns, output='screen')
    yolo_pub_node = Node(package='mini_pjt', executable='yolo_publisher_amr', namespace=ns, output='screen')
    yolo_sub_node = Node(package='mini_pjt', executable='yolo_subscriber_amr', namespace=ns, output='screen')
    yolo_det_node = Node(package='mini_pjt', executable='yolov8_obj_det_amr', namespace=ns, output='screen')
    coordinate_node = Node(package='mini_pjt', executable='coordinate', namespace=ns, output='screen')
    nav_goal_node = Node(package='mini_pjt', executable='my_nav_goal', namespace=ns, output='screen')

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

    # ----------------------------------------------------------------------
    # 3. 실행 순서 제어 (타이머 적용)
    # ----------------------------------------------------------------------
    # Localization, RViz, Nav2가 켜지고 나서 내부 맵 빌드 및 본딩(Bond) 처리가 끝날 때까지 
    # 약 15초의 유예 시간을 준 뒤 내가 만든 프로젝트 노드들을 일제히 실행합니다.
    
    delay_my_project_nodes = TimerAction(
        period=15.0,  # 환경에 따라 10.0 ~ 20.0초 사이로 조절 가능합니다.
        actions=[
            waypoint_node,
            yolo_pub_node,
            yolo_sub_node,
            yolo_det_node,
            coordinate_node,
            cam_to_map_node,
            nav_goal_node
        ]
    )

    # ----------------------------------------------------------------------
    # 4. 최종 반환
    # ----------------------------------------------------------------------
    return LaunchDescription([
        localization_launch,
        viz_launch,
        nav2_launch,
        delay_my_project_nodes  # 15초 뒤 실행될 노드 그룹
    ])