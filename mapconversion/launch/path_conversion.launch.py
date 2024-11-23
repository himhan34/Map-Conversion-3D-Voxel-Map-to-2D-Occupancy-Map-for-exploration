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

            executable='path_converter_node',
            # 실행할 노드의 실행 파일 이름 ('path_converter_node')

            name='path_converter',
            # 노드 이름을 'path_converter'로 설정

            output='screen',
            # 노드의 출력 로그를 화면에 표시

            remappings=[
                # 토픽 이름을 리매핑 (대체 이름으로 연결)
                ('pathIn', '/path'),  # 입력 경로 토픽을 '/path'로 리매핑
                ('pathOut', '/path_3d')  # 출력 경로 토픽을 '/path_3d'로 리매핑
            ],

            parameters=[
                # 노드 실행 시 전달할 매개변수 리스트
                {'use_collision_sphere': True},  # 충돌 영역(구형)을 사용할지 여부 설정
                {'collision_radius': 0.50},  # 충돌 영역의 반지름 설정 (단위: 미터)
                {'path_offset': 1},  # 경로 오프셋 값을 설정
                {'path_smothing_length': 20}  # 경로의 스무딩 길이 설정
            ]
        )
    ])
