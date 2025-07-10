import os #app4.py 实现基本功能版
from flask import Flask, render_template, Response,request,redirect,url_for,session
import cv2
import cv2
import numpy as np
from scipy.special import softmax
# from scipy.special import expit as sigmoid
from hobot_dnn import pyeasy_dnn as dnn  # BSP Python API
import time
import argparse
import logging 
from collections import deque

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
# ====== 新增全局变量 ======
serial_active = True  # 串口激活状态
serial_off_timer = None  # 串口关闭计时器
SERIAL_OFF_DURATION = 3.0  # 串口关闭持续时间（秒）从0.5改为3.0


# ====== 新增：系统状态枚举 ======
class DeviceState:
    IDLE = 0            # 空闲状态：未检测到目标
    TRACKING = 1        # 追踪状态：检测到目标并移动
    PUMPING = 2         # 灌溉状态：水泵工作中
    RECOVERING = 3      # 恢复状态：水泵刚结束，底盘运动激活
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
    "last_position": None,  # 上一次物体位置
    "system_state": DeviceState.IDLE  # 系统状态（新增）
}

# 用于存储最近的状态检测结果
state_history = deque(maxlen=20)  # 存储最近20次检测结果（约1秒数据）
state_change_threshold = 0.80      # 95%的检测结果一致才认为状态改变

# ====== 新增：水泵定时自动复位相关变量 ======
PUMP_ON_DURATION = 5  # 水泵保持时间（秒），后续可修改
pump_timer_start = None  # 记录水泵启动的时间戳
waiting_for_pump_off = False  # 标志：是否等待水泵自动关闭

# ====== 修改设备状态更新函数 ======
def update_device_state(pump, motor, direction, system_state):
    """更新设备状态并发送到串口"""
    global device_state
    if (device_state["pump"] != pump or 
        device_state["motor"] != motor or 
        device_state["direction"] != direction or
        device_state["system_state"] != system_state):
        
        lib.SerialComm_updateStatus(comm, pump, motor, direction)
        device_state = {
            "pump": pump,
            "motor": motor,
            "direction": direction,
            "last_position": device_state["last_position"],
            "system_state": system_state  # 添加系统状态
        }
        logger.info(f"Device State Updated - PUMP:{pump} MOTOR:{motor} DIR:{direction} STATE:{system_state}")

# ========== 状态滤波函数 ==========
def filter_state(current_state):
    """应用状态滤波，确保状态稳定后再更新"""
    global state_history
    
    # 添加当前状态到历史记录
    state_history.append(current_state)
    
    # 如果历史记录不足，不更新状态
    if len(state_history) < 5:
        return current_state
    
    # 计算当前状态在历史记录中的比例
    state_count = sum(1 for state in state_history if state == current_state)
    state_ratio = state_count / len(state_history)
    
    # 如果比例超过阈值，认为状态稳定
    if state_ratio >= state_change_threshold:
        return current_state
    
    # 状态不稳定，保持原状态
    return device_state.get('last_position')

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


def manage_serial_communication(current_target):
    """管理串口通信状态"""
    global serial_active, serial_off_timer, device_state
    
    # 检查是否需要重新激活串口
    if not serial_active and time.time() - serial_off_timer >= SERIAL_OFF_DURATION:
        serial_active = True
        serial_off_timer = None
        
        # 恢复为追踪状态
        if device_state["system_state"] == DeviceState.RECOVERING:
            # 检查是否有目标需要追踪
            if current_target:  # 使用传入的current_target变量
                update_device_state(0, 1, device_state["direction"], DeviceState.TRACKING)
            else:
                update_device_state(0, 0, 0, DeviceState.IDLE)
        
        logger.info("串口通信已重新激活")


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

def analysis_results(ids, scores, bboxes):
    results = []
    for index, bbox in enumerate(bboxes):
        x_min, y_min, x_max, y_max = bbox
        # 存储原始边界框坐标，用于位置判断
        width = x_max - x_min
        height = y_max - y_min
        score = scores[index]
        id = ids[index]
        # 存储边界框坐标而不是中心点
        results.append([id, (x_min, y_min, x_max, y_max), width, height, score])
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
# 用于记录最后拍照时间
last_photo_time = time.time()

def create_img_folder():
    """创建图片保存文件夹，如果imgs存在则创建imgs_1, imgs_2等"""
    base_name = "imgs"
    folder_name = base_name
    counter = 1
    
    while os.path.exists(folder_name):
        folder_name = f"{base_name}_{counter}"
        counter += 1
    
    os.makedirs(folder_name)
    return folder_name

# 创建图片保存文件夹
img_folder = create_img_folder()

def gen_frames():
    global camera, device_state, photo_counter, last_photo_time, img_folder
    global pump_timer_start, waiting_for_pump_off
    global serial_active, serial_off_timer
    
    # Target tracking variables
    current_target = None  # Currently tracked target
    target_lost_counter = 0  # Target lost counter
    MAX_LOST_FRAMES = 10  # Max lost frames (approx 0.1 sec)
    
    # Initialize bounding box coordinates
    x_min, y_min, x_max, y_max = 0, 0, 0, 0

    # System state name mapping
    system_state_names = {
        DeviceState.IDLE: "Idle",
        DeviceState.TRACKING: "Tracking",
        DeviceState.PUMPING: "Pumping",
        DeviceState.RECOVERING: "Recovering"
    }

    while True:
        # ====== Manage serial communication status ======
        manage_serial_communication(current_target)  # Pass current target
        
        with camera_lock:
            if not camera or not camera.isOpened():
                continue
            success, frame = camera.read()

        if not success:
            continue

        # Image processing
        frame = preprocess(frame)
        ids, scores, bboxes = infer_once(frame)
        results = analysis_results(ids, scores, bboxes)
        frame = draw_results(frame, ids, scores, bboxes)

        # Get highest confidence object in current frame
        highest_confidence_result = get_highest_confidence_result(results) if results else None

        # Update current tracking target
        if highest_confidence_result:
            # If new high-confidence target, update tracking target
            current_target = highest_confidence_result
            target_lost_counter = 0
        elif current_target:
            # If no high-confidence target but previous tracking target, increment lost counter
            target_lost_counter += 1
            if target_lost_counter >= MAX_LOST_FRAMES:
                # If lost for multiple frames, reset target
                current_target = None
                target_lost_counter = 0
        
        # ====== Ensure device state is 0 when serial is closed ======
        if not serial_active:
            device_state["pump"] = 0
            device_state["motor"] = 0
            device_state["direction"] = 0
        
        # Position analysis and device control
        current_position = None
        if current_target:
            # Get bounding box info directly from current_target
            # current_target structure: [id, (x_min, y_min, x_max, y_max), width, height, score]
            x_min, y_min, x_max, y_max = current_target[1]  # Bounding box coordinates
            frame_height = frame.shape[0]
            
            # Define middle region boundaries
            middle_upper = frame_height * 0.3  # Upper boundary at 30%
            middle_lower = frame_height * 0.7  # Lower boundary at 70%
            
            # Position determination (using original image coordinates)
            # Entire box in middle region: upper boundary >= 30% and lower boundary <= 70%
            if y_min >= middle_upper and y_max <= middle_lower:
                current_position = 'middle'
            # Upper boundary in upper region: y_min < 30%
            elif y_min < middle_upper:
                current_position = 'up'
            # Lower boundary in lower region: y_max > 70%
            elif y_max > middle_lower:
                current_position = 'down'
            
            # Display tracking target info on screen
            id, bbox, width, height, score = current_target
            cv2.putText(frame, f"Target: ID={id} Conf={score:.2f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            # Display target position
            cv2.putText(frame, f"Position: {current_position if current_position else 'unknown'}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Apply state filtering
        filtered_position = filter_state(current_position) if current_position is not None else None
        
        # ====== Modified pump shutdown logic ======
        if waiting_for_pump_off:
            if time.time() - pump_timer_start >= PUMP_ON_DURATION:
                # Enter recovery state
                update_device_state(0, 0, 0, DeviceState.RECOVERING)  # Recovery state
                logger.info(f"[PUMP TIMER] {PUMP_ON_DURATION}s reached, entering recovery state")
                waiting_for_pump_off = False
                pump_timer_start = None
        
        if filtered_position:
            # Display filtered position info on screen
            cv2.putText(frame, f"Filtered: {filtered_position}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # ====== Modified position analysis logic ======
            if filtered_position == 'up':
                update_device_state(0, 1, 1, DeviceState.TRACKING)  # PUMP_OFF, MOTOR_ON, DIRECTION_UP, Tracking state
                logger.info(f"Tracking state: Moving up - Target position: ({x_min}, {y_min}, {x_max}, {y_max})")
            
            elif filtered_position == 'down':
                update_device_state(0, 1, 0, DeviceState.TRACKING)  # PUMP_OFF, MOTOR_ON, DIRECTION_DOWN, Tracking state
                logger.info(f"Tracking state: Moving down - Target position: ({x_min}, {y_min}, {x_max}, {y_max})")
            
            elif filtered_position == 'middle':
                # Only start pump when entering middle region from tracking state
                if device_state["system_state"] == DeviceState.TRACKING:
                    update_device_state(1, 0, device_state["direction"], DeviceState.PUMPING)  # PUMP_ON, MOTOR_OFF, Pumping state
                    logger.info(f"Pumping state: Starting pump - Target position: ({x_min}, {y_min}, {x_max}, {y_max})")
                    
                    # Start pump timer
                    pump_timer_start = time.time()
                    waiting_for_pump_off = True
            
            # Update last position state
            device_state["last_position"] = filtered_position
        
        else:
            # Enter idle state when no target detected or target lost
            if device_state["system_state"] != DeviceState.IDLE:
                update_device_state(0, 0, 0, DeviceState.IDLE)  # Idle state
                logger.info("No target detected - Entering idle state")
            
            if not current_target:
                cv2.putText(frame, "No target detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "Position unknown", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # ====== Display special status when serial is closed ======
        if not serial_active:
            cv2.putText(frame, "Chassis motion activated", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # Calculate remaining time using SERIAL_OFF_DURATION
            cv2.putText(frame, f"Serial off countdown: {SERIAL_OFF_DURATION - (time.time() - serial_off_timer):.1f}s", 
                       (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Display current device state
        state_text = f"Device: Motor={device_state['motor']} Dir={device_state['direction']} Pump={device_state['pump']}"
        cv2.putText(frame, state_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        
        # ====== Display system state ======
        state_text = f"System State: {system_state_names.get(device_state['system_state'], 'unknown')}"
        cv2.putText(frame, state_text, (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        # Second camera timed photo function
        if using_second_camera and time.time() - last_photo_time > 5:  # Take photo every 5 seconds
            with camera_lock:  # Ensure thread safety
                photo_path = os.path.join(img_folder, f'img_{photo_counter}.jpg')
                cv2.imwrite(photo_path, frame)
                photo_counter += 1
                last_photo_time = time.time()
                logger.info(f"Saved photo to {photo_path}")

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

ACCESS_GRANTED = "access_granted"
app.secret_key = os.urandom(24)  # 生成安全的密钥用于session

@app.before_request
def redirect_to_home():
    """重定向逻辑，避免循环"""
    # 1. 静态文件直接放行
    if request.path.startswith('/static'):
        return
    
    # 2. 访问主页时设置授权标志
    if request.endpoint == 'home':
        session[ACCESS_GRANTED] = True
        return  # 重要：立即返回
    
    # 3. 已授权访问放行
    if session.get(ACCESS_GRANTED):
        return
    
    # 4. 未授权且不是主页 - 重定向到主页
    return redirect(url_for('home'))
    
               
@app.route('/')
def home():
    return render_template('intro.html')

@app.route('/start')
def start():
    if not session.get(ACCESS_GRANTED):
        return redirect(url_for('home'))
    return render_template('index4.html')

@app.route('/about')
def about():
    if not session.get(ACCESS_GRANTED):
        return redirect(url_for('home'))
    return render_template('about.html')


@app.route('/logout')
def logout():
    session[ACCESS_GRANTED] = False
    return redirect(url_for('home'))

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/switch_camera', methods=['POST'])
def switch_camera():
    global camera_index, camera, using_second_camera, last_photo_time
    
    with camera_lock:
        if camera:
            camera.release()
        camera_index = 2 if camera_index == 0 else 0
        using_second_camera = (camera_index == 2)  # 当切换到摄像头2时启用定时拍照
        if using_second_camera:
            last_photo_time = time.time()  # 重置拍照计时器
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            return f"❌ 无法打开摄像头 {camera_index}", 500
    return f"✅ 已切换至摄像头 {camera_index}", 200

import atexit
@atexit.register

# ====== 修改清理函数 ======
def cleanup():
    if 'comm' in globals():
        lib.SerialComm_stop(comm)
    if 'camera' in globals() and camera.isOpened():
        camera.release()
    # ====== 新增：确保串口关闭 ======
    global serial_active
    serial_active = False
    logger.info("System shutdown gracefully")

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        cleanup()