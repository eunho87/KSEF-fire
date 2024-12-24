#!/usr/bin/env python3

# ROS 2 및 관련 라이브러리 임포트
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

# 화재 위치 정보를 구하는 노드 클래스 정의
class FirePositionSubscriber(Node):
    def __init__(self):
        # 노드 이름 설정
        super().__init__('fire_position_subscriber')
        
        # 화재 위치 정보를 구하는 토픽 설정
        self.sub_fire_position = self.create_subscription(
            String,  # 메시지 타입
            'yolov5/fire_position',  # 구독할 토픽 이름
            self.fire_position_callback,  # 콜백 함수
            10  # 큐 사이즈
        )
       
        # 아두이노와의 직렬 통신 설정
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # 시리얼 포트 설정
        self.serial_port.flush()  # 직렬 통신 버퍼 초기화
        time.sleep(2)  # 시리얼 포트 초기화 후 대기 시간
        
        # 상태 추적 변수 초기화
        self.o_active = False  # 'O' 명령어 활성화 여부
        self.p_active = False  # 'P' 명령어 활성화 여부
        self.current_state = 'no_fire'  # 현재 상태 (초기 상태: 'no_fire')

        # 초기 상태에서 'P' 명령 전송 (대기 상태)
        self.send_command_to_arduino('P')  # 'P' 명령어로 대기 상태 진입
        self.p_active = True  # 'P' 상태 활성화 설정

    def send_command_to_arduino(self, command):
        """
        명령어를 아두이노에 전송하고 로그를 기록하는 함수
        Args:
            command (str): 아두이노에 전송할 명령어
        """
        try:
            # 명령어 전송
            self.serial_port.write(command.encode())
            # 전송된 명령어를 로그에 기록
            self.get_logger().info(f"Sent '{command}' command to Arduino.")
        except serial.SerialException as e:
            # 직렬 통신 오류 발생 시 에러 로그 기록
            self.get_logger().error(f"Serial write failed: {e}")

    def fire_position_callback(self, msg: String):
        """
        화재 위치 메시지를 수신했을 때 호출되는 콜백 함수
        Args:
            msg (String): 'yolov5/fire_position' 토픽에서 받은 메시지
        """
        fire_position = msg.data  # 화재 위치 데이터 추출

        # 불이 감지되지 않은 경우 (현재 상태가 'fire_detected'에서 전환)
        if fire_position == 'no_fire' and self.current_state != 'no_fire':
            self.send_command_to_arduino('P')  # 대기 상태 명령 전송
            self.p_active = True  # 'P' 상태 활성화 설정
            if self.o_active:
                self.send_command_to_arduino('S')  # 물 분사 중지 명령 전송
                self.o_active = False  # 'O' 상태 비활성화
            self.current_state = 'no_fire'  # 상태를 'no_fire'로 업데이트

        # 불이 감지된 경우 (현재 상태가 'no_fire'에서 전환)
        elif fire_position != 'no_fire' and self.current_state == 'no_fire':
            self.send_command_to_arduino('F')  # 불 감지 상태 명령 전송
            if not self.o_active:
                self.send_command_to_arduino('O')  # 물 분사 명령 전송
                self.o_active = True  # 'O' 상태 활성화
            self.p_active = False  # 'P' 상태 비활성화
            self.current_state = 'fire_detected'  # 상태를 'fire_detected'로 업데이트

            # 불 위치에 따라 적절한 방향으로 회전 명령 전송
            if fire_position == 'left':
                self.send_command_to_arduino('R')  # 오른쪽으로 회전
            elif fire_position == 'right':
                self.send_command_to_arduino('L')  # 왼쪽으로 회전

# 메인 함수 정의
def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # FirePositionSubscriber 노드 생성 및 실행
    fire_position_subscriber = FirePositionSubscriber()
    rclpy.spin(fire_position_subscriber)  # 노드 실행 대기

    # 실행 종료 후 리소스 정리
    fire_position_subscriber.destroy_node()
    rclpy.shutdown()

# 프로그램 시작점
if __name__ == '__main__':
    main()
