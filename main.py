import argparse  # 명령행 인수를 처리하기 위한 모듈
import os  # 운영체제와 상호작용을 위한 모듈
import sys  # 파이썬 인터프리터와 상호작용을 위한 모듈
from pathlib import Path  # 파일 경로를 다루기 위한 모듈

import cv2  # OpenCV 라이브러리로, 이미지 및 영상 처리에 사용
import numpy as np  # 숫자 배열 계산을 위한 라이브러리

import torch  # PyTorch 딥러닝 프레임워크
import torch.backends.cudnn as cudnn  # CUDA 성능 최적화를 위한 모듈

# YOLOv5 모델 및 관련 유틸리티
from yolov5_ros.models.common import DetectMultiBackend  # 모델 로딩 및 실행
from yolov5_ros.utils.datasets import IMG_FORMATS, VID_FORMATS  # 이미지 및 비디오 포맷
from yolov5_ros.utils.general import (LOGGER, check_img_size, check_imshow, non_max_suppression, scale_coords, xyxy2xywh)
from yolov5_ros.utils.plots import Annotator, colors  # 이미지에 주석 그리기 및 색상 설정
from yolov5_ros.utils.torch_utils import select_device, time_sync  # 디바이스 선택 및 시간 측정 도구
from yolov5_ros.utils.datasets import letterbox  # 입력 이미지를 YOLO 모델에 맞게 전처리

import rclpy  # ROS 2 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 노드 기본 클래스
from sensor_msgs.msg import Image, CompressedImage  # ROS 이미지 메시지 타입
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox  # 바운딩 박스 관련 메시지 타입
from std_msgs.msg import Header, String  # ROS 헤더 및 문자열 메시지 타입
from cv_bridge import CvBridge  # OpenCV 이미지와 ROS 이미지 메시지 간 변환

# YOLOv5 탐지기를 초기화하고 이미지를 처리하는 클래스
class yolov5_demo():
    def __init__(self, weights, data, imagez_height, imagez_width, conf_thres, iou_thres, max_det, device, view_img, classes, agnostic_nms, line_thickness, half, dnn):
        # 입력 파라미터 저장
        self.weights = weights
        self.data = data
        self.imagez_height = imagez_height
        self.imagez_width = imagez_width
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.view_img = view_img
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.line_thickness = line_thickness
        self.half = half
        self.dnn = dnn
        self.s = str()  # 로그 메시지용 문자열

        # 모델 로드
        self.load_model()

    def load_model(self):
        # 입력 이미지 크기 설정
        imgsz = (self.imagez_height, self.imagez_width)

        # 디바이스 선택 (CPU 또는 GPU)
        self.device = select_device(self.device)

        # YOLOv5 모델 로드
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, data=self.data)
        stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        imgsz = check_img_size(imgsz, s=stride)  # 입력 이미지 크기 유효성 검사

        # FP16 지원 여부 확인 및 설정
        self.half &= (pt or jit or onnx or engine) and self.device.type != 'cpu'
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()

        # 모델 초기화 (워밍업)
        bs = 1  # 배치 크기 설정
        self.vid_path, self.vid_writer = [None] * bs, [None] * bs  # 비디오 경로 및 쓰기 설정 초기화
        self.model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # 모델 워밍업

    # 이미지 데이터를 처리하는 콜백 함수
    def image_callback(self, image_raw):
        # 탐지 결과를 저장할 리스트 초기화
        class_list, confidence_list = [], []
        x_min_list, y_min_list, x_max_list, y_max_list = [], [], [], []

        # 이미지를 YOLO 입력 크기에 맞게 전처리
        self.stride = 32  # YOLO 모델의 기본 stride
        self.img_size = 640
        img = letterbox(image_raw, self.img_size, stride=self.stride)[0]

        # 이미지 형식 변환 및 텐서로 변환
        img = img.transpose((2, 0, 1))[::-1]  # HWC -> CHW, BGR -> RGB
        im = np.ascontiguousarray(img)

        t1 = time_sync()  # 처리 시작 시간 기록
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # FP16/FP32 변환
        im /= 255.0  # 정규화 (0-255 -> 0.0-1.0)
        if len(im.shape) == 3:
            im = im[None]  # 배치 차원 추가

        # 모델 추론
        pred = self.model(im, augment=False, visualize=False)

        # 비최대 억제 (NMS) 적용
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # 추론 결과 처리
        for det in pred:
            im0 = image_raw
            if len(det):
                # 바운딩 박스 좌표 및 클래스 정보를 추출
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    class_list.append(self.names[int(cls)])
                    confidence_list.append(conf)
                    x_min_list.append(xyxy[0].item())
                    y_min_list.append(xyxy[1].item())
                    x_max_list.append(xyxy[2].item())
                    y_max_list.append(xyxy[3].item())

        return class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list

# ROS 2 YOLOv5 노드 클래스
class yolov5_ros(Node):
    def __init__(self):
        super().__init__('yolov5_ros')  # 노드 이름 설정

        self.bridge = CvBridge()  # OpenCV와 ROS 이미지 변환 브릿지

        # 퍼블리셔 생성
        self.pub_bbox = self.create_publisher(BoundingBoxes, 'yolov5/bounding_boxes', 10)
        self.pub_image = self.create_publisher(Image, 'yolov5/image_raw', 10)
        self.pub_fire_position = self.create_publisher(String, 'yolov5/fire_position', 10)

        # 구독자 생성 (압축된 이미지 데이터)
        self.sub_image = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        # YOLOv5 탐지기 인스턴스 생성
        self.yolov5 = yolov5_demo(...)

    # 중심 좌표 계산
    def calculate_center(self, x_min, y_min, x_max, y_max):
        return (x_min + x_max) // 2, (y_min + y_max) // 2

    # 화재 위치 결정
    def determine_fire_position(self, center_x, image_width):
        ...

    def image_callback(self, image: CompressedImage):
        # 압축 이미지를 디코딩하고 YOLO 탐지 결과를 처리
        ...
