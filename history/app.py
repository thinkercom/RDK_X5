import os
from flask import Flask, render_template, Response
import cv2
import cv2
import numpy as np
from scipy.special import softmax
# from scipy.special import expit as sigmoid
from hobot_dnn import pyeasy_dnn as dnn  # BSP Python API
import time
import argparse
import logging 

import cv2
import numpy as np
from scipy.special import softmax
# from scipy.special import expit as sigmoid
from hobot_dnn import pyeasy_dnn as dnn  # BSP Python API
import time
import argparse
import logging 

import serial
from serial import Serial
import threading
import ctypes

# 确保程序退出时释放资源
import atexit

# 加载动态链接库
lib = ctypes.CDLL('/home/sunrise/Project20250627/serial/libserialcomm.so')

# 定义函数原型
lib.SerialComm_new.argtypes = [ctypes.c_char_p]
lib.SerialComm_new.restype = ctypes.c_void_p
lib.SerialComm_start.argtypes = [ctypes.c_void_p]
lib.SerialComm_start.restype = ctypes.c_int
lib.SerialComm_stop.argtypes = [ctypes.c_void_p]
lib.SerialComm_updateStatus.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8]

# 初始化串口通信
serial_port = b'/dev/ttyS1'
comm = lib.SerialComm_new(serial_port)
if lib.SerialComm_start(comm) != 0:
    logger.error("Failed to start serial communication.")
    exit(1)

# 设备状态管理
device_state = {
    "pump": 0,       # 水泵状态 0:OFF 1:ON
    "motor": 0,      # 马达状态 0:OFF 1:ON
    "direction": 0,  # 方向 0:DOWN 1:UP
    "last_position": None  # 上一次物体位置
}

def update_device_state(pump, motor, direction):
    """更新设备状态并发送到串口"""
    global device_state
    if (device_state["pump"] != pump or 
        device_state["motor"] != motor or 
        device_state["direction"] != direction):
        
        lib.SerialComm_updateStatus(comm, pump, motor, direction)
        device_state = {
            "pump": pump,
            "motor": motor,
            "direction": direction,
            "last_position": device_state["last_position"]
        }
        logger.info(f"Device State Updated - PUMP:{pump} MOTOR:{motor} DIR:{direction}")

def analyze_position(frame, bboxes, scores):
    """分析物体位置并返回位置状态"""
    if len(bboxes) == 0:
        return None
    
    # 获取置信度最高的检测结果
    max_idx = np.argmax(scores)
    x1, y1, x2, y2 = bboxes[max_idx]
    
    # 计算物体中心位置和高度
    object_y_center = (y1 + y2) / 2
    object_height = y2 - y1
    frame_height = frame.shape[0]
    
    # 动态区域划分（考虑物体高度）
    upper_bound = frame_height * 0.4
    lower_bound = frame_height * 0.6
    middle_lower = (frame_height - object_height) * 0.4
    middle_upper = (frame_height + object_height) * 0.6
    
    # 位置判断
    if object_y_center < upper_bound:
        return 'up'
    elif object_y_center > lower_bound:
        return 'down'
    elif middle_lower <= object_y_center <= middle_upper:
        return 'middle'
    return None

# logging configs
logging.basicConfig(
    level = logging.DEBUG,
    format = '[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S')
logger = logging.getLogger("RDK_YOLO")

class BaseModel:
    def __init__(
        self,
        model_file: str
        ) -> None:
        # 加载BPU的bin模型, 打印相关参数
        # Load the quantized *.bin model and print its parameters
        try:
            begin_time = time.time()
            self.quantize_model = dnn.load(model_file)
            logger.debug("\033[1;31m" + "Load D-Robotics Quantize model time = %.2f ms"%(1000*(time.time() - begin_time)) + "\033[0m")
        except Exception as e:
            logger.error("❌ Failed to load model file: %s"%(model_file))
            logger.error("You can download the model file from the following docs: ./models/download.md") 
            logger.error(e)
            exit(1)

        logger.info("\033[1;32m" + "-> input tensors" + "\033[0m")
        for i, quantize_input in enumerate(self.quantize_model[0].inputs):
            logger.info(f"intput[{i}], name={quantize_input.name}, type={quantize_input.properties.dtype}, shape={quantize_input.properties.shape}")

        logger.info("\033[1;32m" + "-> output tensors" + "\033[0m")
        for i, quantize_input in enumerate(self.quantize_model[0].outputs):
            logger.info(f"output[{i}], name={quantize_input.name}, type={quantize_input.properties.dtype}, shape={quantize_input.properties.shape}")

        self.model_input_height, self.model_input_weight = self.quantize_model[0].inputs[0].properties.shape[2:4]

    def resizer(self, img: np.ndarray)->np.ndarray:
        img_h, img_w = img.shape[0:2]
        self.y_scale, self.x_scale = img_h/self.model_input_height, img_w/self.model_input_weight
        return cv2.resize(img, (self.model_input_height, self.model_input_weight), interpolation=cv2.INTER_NEAREST) # 利用resize重新开辟内存
    
    def preprocess(self, img: np.ndarray)->np.array:
        """
        Preprocesses an input image to prepare it for model inference.

        Args:
            img (np.ndarray): The input image in BGR format as a NumPy array.

        Returns:
            np.array: The preprocessed image tensor in NCHW format ready for model input.

        Procedure:
            1. Resizes the image to a specified dimension (`input_image_size`) using nearest neighbor interpolation.
            2. Converts the image color space from BGR to RGB.
            3. Transposes the dimensions of the image tensor to channel-first order (CHW).
            4. Adds a batch dimension, thus conforming to the NCHW format expected by many models.
            Note: Normalization to [0, 1] is assumed to be handled elsewhere based on configuration.
        """
        begin_time = time.time()

        input_tensor = self.resizer(img)
        input_tensor = cv2.cvtColor(input_tensor, cv2.COLOR_BGR2RGB)
        # input_tensor = np.array(input_tensor) / 255.0  # yaml文件中已经配置前处理
        input_tensor = np.transpose(input_tensor, (2, 0, 1))
        input_tensor = np.expand_dims(input_tensor, axis=0).astype(np.uint8)  # NCHW

        logger.debug("\033[1;31m" + f"pre process time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        return input_tensor

    def bgr2nv12(self, bgr_img: np.ndarray) -> np.ndarray:
        """
        Convert a BGR image to the NV12 format.

        NV12 is a common video encoding format where the Y component (luminance) is full resolution,
        and the UV components (chrominance) are half-resolution and interleaved. This function first
        converts the BGR image to YUV 4:2:0 planar format, then rearranges the UV components to fit
        the NV12 format.

        Parameters:
        bgr_img (np.ndarray): The input BGR image array.

        Returns:
        np.ndarray: The converted NV12 format image array.
        """
        begin_time = time.time()
        bgr_img = self.resizer(bgr_img)
        height, width = bgr_img.shape[0], bgr_img.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.zeros_like(yuv420p)
        nv12[:height * width] = y
        nv12[height * width:] = uv_packed

        logger.debug("\033[1;31m" + f"bgr8 to nv12 time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        return nv12


    def forward(self, input_tensor: np.array) -> list[dnn.pyDNNTensor]:
        begin_time = time.time()
        quantize_outputs = self.quantize_model[0].forward(input_tensor)
        logger.debug("\033[1;31m" + f"forward time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        return quantize_outputs


    def c2numpy(self, outputs) -> list[np.array]:
        begin_time = time.time()
        outputs = [dnnTensor.buffer for dnnTensor in outputs]
        logger.debug("\033[1;31m" + f"c to numpy time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        return outputs

class YOLO11_Detect(BaseModel):
    def __init__(self, 
                model_file: str, 
                conf: float, 
                iou: float
                ):
        super().__init__(model_file)
        # 将反量化系数准备好, 只需要准备一次
        # prepare the quantize scale, just need to generate once
        self.s_bboxes_scale = self.quantize_model[0].outputs[0].properties.scale_data[np.newaxis, :]
        self.m_bboxes_scale = self.quantize_model[0].outputs[1].properties.scale_data[np.newaxis, :]
        self.l_bboxes_scale = self.quantize_model[0].outputs[2].properties.scale_data[np.newaxis, :]
        logger.info(f"{self.s_bboxes_scale.shape=}, {self.m_bboxes_scale.shape=}, {self.l_bboxes_scale.shape=}")

        # DFL求期望的系数, 只需要生成一次
        # DFL calculates the expected coefficients, which only needs to be generated once.
        self.weights_static = np.array([i for i in range(16)]).astype(np.float32)[np.newaxis, np.newaxis, :]
        logger.info(f"{self.weights_static.shape = }")

        # anchors, 只需要生成一次
        self.s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                            np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
        self.m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                            np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
        self.l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                            np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)
        logger.info(f"{self.s_anchor.shape = }, {self.m_anchor.shape = }, {self.l_anchor.shape = }")

        # 输入图像大小, 一些阈值, 提前计算好
        self.input_image_size = 640
        self.conf = conf
        self.iou = iou
        self.conf_inverse = -np.log(1/conf - 1)
        logger.info("iou threshol = %.2f, conf threshol = %.2f"%(iou, conf))
        logger.info("sigmoid_inverse threshol = %.2f"%self.conf_inverse)
    

    def postProcess(self, outputs: list[np.ndarray]) -> tuple[list]:
        begin_time = time.time()
        # reshape
        s_bboxes = outputs[0].reshape(-1, 64)
        m_bboxes = outputs[1].reshape(-1, 64)
        l_bboxes = outputs[2].reshape(-1, 64)
        s_clses = outputs[3].reshape(-1, 80)
        m_clses = outputs[4].reshape(-1, 80)
        l_clses = outputs[5].reshape(-1, 80)

        # classify: 利用numpy向量化操作完成阈值筛选(优化版 2.0)
        s_max_scores = np.max(s_clses, axis=1)
        s_valid_indices = np.flatnonzero(s_max_scores >= self.conf_inverse)  # 得到大于阈值分数的索引，此时为小数字
        s_ids = np.argmax(s_clses[s_valid_indices, : ], axis=1)
        s_scores = s_max_scores[s_valid_indices]

        m_max_scores = np.max(m_clses, axis=1)
        m_valid_indices = np.flatnonzero(m_max_scores >= self.conf_inverse)  # 得到大于阈值分数的索引，此时为小数字
        m_ids = np.argmax(m_clses[m_valid_indices, : ], axis=1)
        m_scores = m_max_scores[m_valid_indices]

        l_max_scores = np.max(l_clses, axis=1)
        l_valid_indices = np.flatnonzero(l_max_scores >= self.conf_inverse)  # 得到大于阈值分数的索引，此时为小数字
        l_ids = np.argmax(l_clses[l_valid_indices, : ], axis=1)
        l_scores = l_max_scores[l_valid_indices]

        # 3个Classify分类分支：Sigmoid计算
        s_scores = 1 / (1 + np.exp(-s_scores))
        m_scores = 1 / (1 + np.exp(-m_scores))
        l_scores = 1 / (1 + np.exp(-l_scores))

        # 3个Bounding Box分支：筛选
        s_bboxes_float32 = s_bboxes[s_valid_indices,:]#.astype(np.float32) * self.s_bboxes_scale
        m_bboxes_float32 = m_bboxes[m_valid_indices,:]#.astype(np.float32) * self.m_bboxes_scale
        l_bboxes_float32 = l_bboxes[l_valid_indices,:]#.astype(np.float32) * self.l_bboxes_scale

        # 3个Bounding Box分支：dist2bbox (ltrb2xyxy)
        s_ltrb_indices = np.sum(softmax(s_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        s_anchor_indices = self.s_anchor[s_valid_indices, :]
        s_x1y1 = s_anchor_indices - s_ltrb_indices[:, 0:2]
        s_x2y2 = s_anchor_indices + s_ltrb_indices[:, 2:4]
        s_dbboxes = np.hstack([s_x1y1, s_x2y2])*8

        m_ltrb_indices = np.sum(softmax(m_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        m_anchor_indices = self.m_anchor[m_valid_indices, :]
        m_x1y1 = m_anchor_indices - m_ltrb_indices[:, 0:2]
        m_x2y2 = m_anchor_indices + m_ltrb_indices[:, 2:4]
        m_dbboxes = np.hstack([m_x1y1, m_x2y2])*16

        l_ltrb_indices = np.sum(softmax(l_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        l_anchor_indices = self.l_anchor[l_valid_indices,:]
        l_x1y1 = l_anchor_indices - l_ltrb_indices[:, 0:2]
        l_x2y2 = l_anchor_indices + l_ltrb_indices[:, 2:4]
        l_dbboxes = np.hstack([l_x1y1, l_x2y2])*32

        # 大中小特征层阈值筛选结果拼接
        dbboxes = np.concatenate((s_dbboxes, m_dbboxes, l_dbboxes), axis=0)
        scores = np.concatenate((s_scores, m_scores, l_scores), axis=0)
        ids = np.concatenate((s_ids, m_ids, l_ids), axis=0)

        # nms
        indices = cv2.dnn.NMSBoxes(dbboxes, scores, self.conf, self.iou)

        # 还原到原始的img尺度
        bboxes = dbboxes[indices] * np.array([self.x_scale, self.y_scale, self.x_scale, self.y_scale])
        bboxes = bboxes.astype(np.int32)

        logger.debug("\033[1;31m" + f"Post Process time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")

        return ids[indices], scores[indices], bboxes
    

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def softmax(x, axis=-1):
    e_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
    return e_x / np.sum(e_x, axis=axis, keepdims=True)
def post_process(outputs, anchors, strides, weights_static, conf_thres=0.25, iou_thres=0.5, scale_factors=(1.0, 1.0)):
    """
    outputs: list of 6 np.ndarray:
        [0] (1, 80, 80, 64), [1] (1, 40, 40, 64), [2] (1, 20, 20, 64) -- bbox
        [3] (1, 80, 80, 1),  [4] (1, 40, 40, 1),  [5] (1, 20, 20, 1)  -- cls (1类)
    anchors: list of 3 (N, 2) arrays for s/m/l anchor center coords
    strides: list of 3 stride ints, e.g. [8, 16, 32]
    weights_static: (1, 16) array for bbox解码 softmax 加权
    """

    dbboxes_all, scores_all = [], []

    for i in range(3):  # 遍历三个特征层
        bbox_pred = outputs[i].reshape(-1, 64)         # shape: (H*W*A, 64)
        cls_pred = outputs[i + 3].reshape(-1, 1)       # shape: (H*W*A, 1)
        anchors_grid = anchors[i]                     # shape: (H*W*A, 2)
        stride = strides[i]

        # 分类得分（Sigmoid + 阈值）
        scores = sigmoid(cls_pred.squeeze())          # shape: (N,)
        valid_mask = scores >= conf_thres
        if np.sum(valid_mask) == 0:
            continue

        bbox_valid = bbox_pred[valid_mask]
        scores_valid = scores[valid_mask]
        anchors_valid = anchors_grid[valid_mask]

        # bbox解码：reshape为 (N, 4, 16)，softmax → weighted sum
        ltrb_distri = bbox_valid.reshape(-1, 4, 16)
        ltrb = np.sum(softmax(ltrb_distri, axis=2) * weights_static, axis=2)

        # 解码得到 x1, y1, x2, y2（ltrb 相对 anchor 中心）
        x1y1 = anchors_valid - ltrb[:, :2]
        x2y2 = anchors_valid + ltrb[:, 2:]
        boxes = np.hstack([x1y1, x2y2]) * stride

        dbboxes_all.append(boxes)
        scores_all.append(scores_valid)

    if not dbboxes_all:
        return [], [], []

    dbboxes_all = np.concatenate(dbboxes_all, axis=0)
    scores_all = np.concatenate(scores_all, axis=0)
    ids_all = np.zeros_like(scores_all, dtype=int)  # 全是0类

    # NMS：OpenCV 格式是 [x, y, w, h]
    xywh = dbboxes_all.copy()
    xywh[:, 2:] -= xywh[:, :2]  # 转换为 w, h

    nms_indices = cv2.dnn.NMSBoxes(
        bboxes=xywh.tolist(), scores=scores_all.tolist(),
        score_threshold=conf_thres, nms_threshold=iou_thres
    )

    if len(nms_indices) == 0:
        return [], [], []

    nms_indices = np.array(nms_indices).flatten()

    x_scale, y_scale = scale_factors
    scale = np.array([x_scale, y_scale, x_scale, y_scale])
    final_boxes = dbboxes_all[nms_indices] * scale
    final_boxes = final_boxes.astype(np.int32)

    return ids_all[nms_indices], scores_all[nms_indices], final_boxes


def infer_once(frame):
    s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                            np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
    m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                                np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
    l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                                np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)
    input_tensor=model.bgr2nv12(frame)
    outputs = model.c2numpy(model.forward(input_tensor))
    weights_static = np.arange(16, dtype=np.float32).reshape(1, 1, 16)
    strides = [8, 16, 32]
    ids,scores,bboxes=post_process(outputs,[s_anchor,m_anchor,l_anchor],strides,weights_static)
    return ids,scores,bboxes

def analysis_results(ids,scores,bboxes):
    results=[]
    for index,bbox in enumerate(bboxes):
        x_min,y_min,x_max,y_max=bbox
        center=((x_min+x_max)/2-320,320-(y_min+y_max)/2)
        width=x_max-x_min
        height=y_max-y_min
        score=scores[index]
        id=ids[index]
        results.append([id,center,width,height,score])
    return results
def draw_results(frame, ids, scores, bboxes, color=(0, 255, 0), thickness=2):
    """
    在图像上绘制目标检测的结果，包括边框、类别ID和得分。

    参数:
        frame (np.ndarray): 原始图像（BGR格式）
        ids (np.ndarray): 检测到的类别ID数组
        scores (np.ndarray): 每个框的置信度分数（0~1之间）
        bboxes (np.ndarray): 每个框的坐标，形状为 (N, 4)，格式 [x1, y1, x2, y2]
        color (tuple): 边框颜色，默认绿色
        thickness (int): 边框粗细
    返回:
        np.ndarray: 带绘制结果的图像
    """
    for cls_id, score, box in zip(ids, scores, bboxes):
        x1, y1, x2, y2 = box
        label = f"{int(cls_id)}: {score:.2f}"

        # 画框
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        # 画文本背景
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_w, text_h = text_size
        cv2.rectangle(frame, (x1, y1 - text_h - 4), (x1 + text_w + 2, y1), color, -1)

        # 写文字
        cv2.putText(frame, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return frame

def get_highest_confidence_result(detections):
    """
    从目标检测结果中保留置信度最高的一个结果。

    :param detections: List of detection results, each in format
                       [class_id, (x_center, y_center), width, height, confidence]
    :return: Detection result with the highest confidence, or None if input is empty
    """
    if not detections:
        return None
    return max(detections, key=lambda x: x[4])


def preprocess(frame):
    """
    预处理图像：letterbox 缩放 + padding
    返回 input_tensor, 缩放比例, padding 偏移
    """
    orig_h, orig_w = frame.shape[:2]
    target_w, target_h = 640,640
    scale = min(target_w / orig_w, target_h / orig_h)

    new_w = int(orig_w * scale)
    new_h = int(orig_h * scale)
    resized = cv2.resize(frame, (new_w, new_h))

    pad_w = target_w - new_w
    pad_h = target_h - new_h
    top = pad_h // 2
    bottom = pad_h - top
    left = pad_w // 2
    right = pad_w - left
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded


def log_info(content, log_file="log.txt"):
    """
    记录并打印日志信息
    :param content: 日志内容
    :param log_file: 日志文件路径，默认为当前目录下的 log.txt
    """
    # 获取当前时间
    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    
    # 格式化日志信息
    log_message = f"[{current_time}] {content}"
    
    # 打印日志到控制台
    print(log_message)
    
    # 将日志写入文件
    # with open(log_file, "a", encoding="utf-8") as file:
    #     file.write(log_message + "\n")


logging.basicConfig(
    level = logging.DEBUG,
    format = '[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S')
logger = logging.getLogger("RDK_YOLO")

model_path="/home/sunrise/Project20250627/lajiao.bin"
iou_thres=0.45
conf_thres=0.25
classes_num=1
reg=16
model = YOLO11_Detect(model_path, conf_thres, iou_thres)

app = Flask(__name__)

# 线程锁 + 当前摄像头索引和对象
camera_index = 0
camera = cv2.VideoCapture(camera_index)
camera_lock = threading.Lock()

# 用于记录拍照的计数器
photo_counter = 1
# 用于记录是否使用第二个摄像头
using_second_camera = False

# 创建imgs文件夹
if not os.path.exists('imgs'):
    os.makedirs('imgs')

def gen_frames():
    global camera,device_state
    while True:
        with camera_lock:
            if not camera or not camera.isOpened():
                continue
            success, frame = camera.read()

        if not success:
            continue

        # 图像处理
        frame=preprocess(frame)
        ids,scores,bboxes=infer_once(frame)
        results=analysis_results(ids,scores,bboxes)
        frame=draw_results(frame,ids,scores,bboxes)

        # 位置分析和设备控制
        current_position = analyze_position(frame, bboxes, scores)

        if current_position:
            # 在画面上显示位置信息
            cv2.putText(frame, f"Position: {current_position}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 根据位置更新设备状态
            if current_position == 'up':
                update_device_state(0, 1, 1)  # PUMP_OFF, MOTOR_ON, DIRECTION_UP
            
            elif current_position == 'down':
                update_device_state(0, 1, 0)  # PUMP_OFF, MOTOR_ON, DIRECTION_DOWN
            
            elif current_position == 'middle':
                # 只有从上下方向进入中间区域时才开启水泵
                if device_state["last_position"] in ['up', 'down']:
                    update_device_state(1, 0, device_state["direction"])  # PUMP_ON, MOTOR_OFF
            
            # 更新最后位置状态
            device_state["last_position"] = current_position
        
        else:
            # 没有检测到物体时停止所有设备
            if device_state["motor"] == 1 or device_state["pump"] == 1:
                update_device_state(0, 0, 0)  # 全部停止
            
            cv2.putText(frame, "No object detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # 显示当前设备状态
        state_text = f"State: Motor={device_state['motor']} Dir={device_state['direction']} Pump={device_state['pump']}"
        cv2.putText(frame, state_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        # 如果使用第二个摄像头，定时拍照
        if using_second_camera and time.time() - last_photo_time > 5:  # 每5秒拍一次照
            photo_path = os.path.join('imgs', f'result_{photo_counter}.jpg')
            cv2.imwrite(photo_path, frame)
            photo_counter += 1
            last_photo_time = time.time()

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame = buffer.tobytes()


        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index3.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/switch_camera', methods=['POST'])
def switch_camera():
    global camera_index, camera
    with camera_lock:
        if camera:
            camera.release()
        camera_index = 2 if camera_index == 0 else 0
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            return f"❌ 无法打开摄像头 {camera_index}", 500
    return f"✅ 已切换至摄像头 {camera_index}", 200

import atexit
@atexit.register
def cleanup():
    if 'comm' in globals():
        lib.SerialComm_stop(comm)
    if 'camera' in globals() and camera.isOpened():
        camera.release()
    logger.info("System shutdown gracefully")

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        cleanup()