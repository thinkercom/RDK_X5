# utils.py
import os
import time
from collections import deque
import cv2
import numpy as np

state_history = deque(maxlen=20)
state_change_threshold = 0.80

def filter_state(current_state):
    global state_history
    state_history.append(current_state)
    if len(state_history) < 5:
        return current_state
    state_count = sum(1 for state in state_history if state == current_state)
    state_ratio = state_count / len(state_history)
    if state_ratio >= state_change_threshold:
        return current_state
    return None

def create_img_folder():
    base_dir = "imgs"
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    counter = 1
    while True:
        sub_folder = os.path.join(base_dir, f"imgs_{counter}")
        if not os.path.exists(sub_folder):
            os.makedirs(sub_folder)
            return sub_folder
        counter += 1

def get_targets_in_x_center(results, frame_width, center_ratio=0.4):
    center_margin = (1.0 - center_ratio) / 2
    left_center = frame_width * center_margin
    right_center = frame_width * (1.0 - center_margin)
    in_center_targets = []
    for r in results:
        x_min, y_min, x_max, y_max = r[1]
        if x_min < right_center and x_max > left_center:
            in_center_targets.append(r)
    return in_center_targets

def log_info(content, log_file="log.txt"):
    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    log_message = f"[{current_time}] {content}"
    print(log_message)
    # with open(log_file, "a", encoding="utf-8") as file:
    #     file.write(log_message + "\\n")

def preprocess(frame):
    """
    预处理图像：letterbox 缩放 + padding
    返回处理后的图像
    """
    orig_h, orig_w = frame.shape[:2]
    target_w, target_h = 640, 640
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

def draw_results(frame, ids, scores, bboxes, color=(0, 255, 0), thickness=2):
    """
    在图像上绘制目标检测的结果，包括边框、类别ID和得分。
    """
    for cls_id, score, box in zip(ids, scores, bboxes):
        x1, y1, x2, y2 = box
        label = f"{int(cls_id)}: {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_w, text_h = text_size
        cv2.rectangle(frame, (x1, y1 - text_h - 4), (x1 + text_w + 2, y1), color, -1)
        cv2.putText(frame, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return frame