#!/usr/bin/env python3

# ROS 2 및 Python 직렬 통신 라이브러리 임포트
import rclpy
from rclpy.node import Node
import serial
import time

# ROS 2 노드 클래스 정의
class SerialCommandNode(Node):
    def __init__(self):
        # 노드 이름 설정
        super().__init__('serial_command_node')
        
        # 아두이노와의 직렬 통신 설정
        try:
            # /dev/ttyACM0 포트를 9600 baud rate로 설정
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.serial_port.flush()  # 직렬 통신 버퍼 초기화
        except serial.SerialException as e:
            # 직렬 포트 연결 실패 시 에러 메시지 출력
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return

        # 0.5초 간격으로 check_input 함수를 호출하는 타이머 생성
        self.timer = self.create_timer(0.5, self.check_input)

    def send_command_to_arduino(self, command):
        """
        명령어를 아두이노에 전송하고 응답을 기다리는 함수
        Args:
            command (str): 아두이노에 전송할 명령어
        """
        try:
            # 명령어 전송
            self.serial_port.write(command.encode())
            # 전송 후 100ms 대기 (아두이노가 데이터를 처리할 시간)
            time.sleep(0.1)
            # 응답 읽기
            response = self.serial_port.read_until().decode('utf-8').strip()
            # 응답 메시지 출력
            self.get_logger().info(f"Arduino response: {response}")
        except Exception as e:
            # 직렬 통신 오류 처리
            self.get_logger().error(f"Error during serial communication: {e}")

    def check_input(self):
        """
        주기적으로 사용자 입력을 확인하고 명령어를 처리하는 함수
        """
        try:
            # 사용자 입력 대기
            user_input = input("Enter command (O, R, L, P, F, R+O, L+O or 'exit' to quit): ")

            # 'exit' 입력 시 노드 종료
            if user_input == 'exit':
                self.get_logger().info("Exiting...")
                rclpy.shutdown()
            # 단일 명령어 처리
            elif user_input in ['O', 'R', 'L', 'P', 'F']:
                self.send_command_to_arduino(user_input)
            # 복합 명령어 처리 (R+O)
            elif user_input == 'R+O':
                self.get_logger().info("Sending R and O commands...")
                self.send_command_to_arduino('R')
                self.send_command_to_arduino('O')
            # 복합 명령어 처리 (L+O)
            elif user_input == 'L+O':
                self.get_logger().info("Sending L and O commands...")
                self.send_command_to_arduino('L')
                self.send_command_to_arduino('O')
            else:
                # 유효하지 않은 명령어 입력 시 경고
                self.get_logger().info("Invalid command. Please enter 'O', 'R', 'L', 'P', 'F', 'R+O', 'L+O' or 'exit'.")
                
        except EOFError:
            # EOFError 처리 (예: Ctrl+D 입력 시)
            self.get_logger().info("EOF detected. Shutting down...")
            rclpy.shutdown()

# 메인 함수 정의
def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # SerialCommandNode 객체 생성 및 실행
    serial_command_node = SerialCommandNode()
    rclpy.spin(serial_command_node)  # 노드 실행 대기
    
    # 실행 종료 후 리소스 정리
    serial_command_node.destroy_node()
    rclpy.shutdown()

# 프로그램 시작점
if __name__ == '__main__':
    main()
