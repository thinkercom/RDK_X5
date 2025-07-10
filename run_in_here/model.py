# model.py
import numpy as np
import cv2
from hobot_dnn import pyeasy_dnn as dnn
import time
import logging

class YOLO11_Detect:
    def __init__(self, model_file: str, conf: float, iou: float):
        try:
            begin_time = time.time()
            self.quantize_model = dnn.load(model_file)
            print(f"Load D-Robotics Quantize model time = {1000*(time.time() - begin_time):.2f} ms")
        except Exception as e:
            print(f"❌ Failed to load model file: {model_file}")
            print(e)
            exit(1)
        self.model_input_height, self.model_input_weight = self.quantize_model[0].inputs[0].properties.shape[2:4]
        self.s_bboxes_scale = self.quantize_model[0].outputs[0].properties.scale_data[np.newaxis, :]
        self.m_bboxes_scale = self.quantize_model[0].outputs[1].properties.scale_data[np.newaxis, :]
        self.l_bboxes_scale = self.quantize_model[0].outputs[2].properties.scale_data[np.newaxis, :]
        self.s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                                  np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
        self.m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                                  np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
        self.l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                                  np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)
        self.input_image_size = 640
        self.conf = conf
        self.iou = iou
        self.conf_inverse = -np.log(1/conf - 1)
        self.weights_static = np.array([i for i in range(16)]).astype(np.float32)[np.newaxis, np.newaxis, :]

    def resizer(self, img: np.ndarray)->np.ndarray:
        img_h, img_w = img.shape[0:2]
        self.y_scale, self.x_scale = img_h/self.model_input_height, img_w/self.model_input_weight
        return cv2.resize(img, (self.model_input_height, self.model_input_weight), interpolation=cv2.INTER_NEAREST)
    
    def preprocess(self, img: np.ndarray)->np.array:
        input_tensor = self.resizer(img)
        input_tensor = cv2.cvtColor(input_tensor, cv2.COLOR_BGR2RGB)
        input_tensor = np.transpose(input_tensor, (2, 0, 1))
        input_tensor = np.expand_dims(input_tensor, axis=0).astype(np.uint8)
        return input_tensor

    def bgr2nv12(self, bgr_img: np.ndarray) -> np.ndarray:
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
        return nv12

    def forward(self, input_tensor: np.array):
        return self.quantize_model[0].forward(input_tensor)

    def c2numpy(self, outputs):
        return [dnnTensor.buffer for dnnTensor in outputs]

def infer_once(frame):
    s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                        np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
    m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                        np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
    l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                        np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)
    # 这里假设model是全局变量
    input_tensor = model.bgr2nv12(frame)
    outputs = model.c2numpy(model.forward(input_tensor))
    weights_static = np.arange(16, dtype=np.float32).reshape(1, 1, 16)
    strides = [8, 16, 32]
    ids, scores, bboxes = post_process(outputs, [s_anchor, m_anchor, l_anchor], strides, weights_static)
    return ids, scores, bboxes

def analysis_results(ids, scores, bboxes):
    results = []
    for index, bbox in enumerate(bboxes):
        x_min, y_min, x_max, y_max = bbox
        width = x_max - x_min
        height = y_max - y_min
        score = scores[index]
        id = ids[index]
        results.append([id, (x_min, y_min, x_max, y_max), width, height, score])
    return results

# 也可以把sigmoid、softmax、post_process等辅助函数也放在这里