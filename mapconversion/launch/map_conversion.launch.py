from launch import LaunchDescription
# ROS 2 Launch 시스템에서 LaunchDescription 클래스를 가져옴

from launch_ros.actions import Node
# ROS 2 노드 실행을 위한 Node 액션 클래스를 가져옴

def generate_launch_description():
    # Launch 파일의 엔트리 포인트가 되는 함수 정의
    return LaunchDescription([
        # LaunchDescription 객체를 생성하여 반환
        Node(
            package='mapconversion',
            # 실행할 노드가 포함된 패키지 이름 ('mapconversion')

            executable='map_conversion_oct_node',
            # 실행할 노드의 실행 파일 이름 ('map_conversion_oct_node')

            name='map_conversion',
            # 노드 이름을 'map_conversion'으로 설정

            output='screen',
            # 노드의 출력 로그를 화면에 표시

            parameters=[
                # 노드 실행 시 전달할 매개변수 리스트
                {'minimum_z': 1.0},  # Z 축의 최소값 설정
                {'minimum_occupancy': 10},  # 최소 점유 상태 값 설정
                {'map_frame': 'map'},  # 맵 프레임 이름 설정
                {'max_slope_ugv': 0.2},  # 최대 경사 값 설정
                {'slope_estimation_size': 2}  # 경사 추정 크기 설정
            ],
            remappings=[
                # 토픽 이름을 리매핑 (대체 이름으로 연결)
                ('octomap', 'octomap_binary')  # 'octomap' 토픽을 'octomap_binary'로 변경
            ]
        )
    ])
