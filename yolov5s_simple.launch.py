import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch 파일에서 실행될 노드를 정의하는 함수
def generate_launch_description():
    # yolov5_ros 패키지의 공유 디렉토리 경로를 가져옵니다.
    yolox_ros_share_dir = get_package_share_directory('yolov5_ros')

    # 웹캠 노드 설정
    webcam = launch_ros.actions.Node(
        package="v4l2_camera",  # 사용할 ROS2 패키지 이름
        executable="v4l2_camera_node",  # 실행할 노드 이름
        parameters=[  # 노드에 전달할 파라미터 설정
            {"image_size": [640,480]},  # 웹캠 출력 이미지 크기 (가로 640, 세로 480)
        ],
    )

    # YOLOv5 ROS 노드 설정
    yolov5_ros = launch_ros.actions.Node(
        package="yolov5_ros",  # 사용할 ROS2 패키지 이름
        executable="yolov5_ros",  # 실행할 노드 이름
        parameters=[  # 노드에 전달할 파라미터 설정
            {"view_img": True},  # 이미지 추론 결과를 화면에 표시
        ],
    )

    # rqt_graph 노드 설정 (노드 간 연결 관계를 시각화하는 툴)
    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph",  # 사용할 ROS2 패키지 이름
        executable="rqt_graph",  # 실행할 노드 이름
    )

    # launch 파일에서 실행될 모든 노드들을 반환
    return launch.LaunchDescription([
        webcam,  # 웹캠 노드를 실행
        yolov5_ros,  # YOLOv5 추론 노드를 실행
        rqt_graph,  # rqt_graph 시각화 툴을 실행
    ])
