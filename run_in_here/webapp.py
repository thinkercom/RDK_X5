# webapp.py
from flask import Flask, render_template, Response, request, redirect, url_for, session
import cv2
import threading
import time
from config import *
from device import device_state, update_device_state
from model import YOLO11_Detect, infer_once, analysis_results
from utils import create_img_folder, filter_state, get_targets_in_x_center, preprocess, draw_results

app = Flask(__name__)
app.secret_key = "your_secret_key"

camera_index = 0
camera = cv2.VideoCapture(camera_index)
camera_lock = threading.Lock()
photo_counter = 1
using_second_camera = False
last_photo_time = time.time()
img_folder = create_img_folder()
pump_timer_start = None
waiting_for_pump_off = False
serial_active = True
serial_off_timer = None

def gen_frames():
    global camera, device_state, photo_counter, last_photo_time, img_folder
    global pump_timer_start, waiting_for_pump_off
    global serial_active, serial_off_timer

    current_target = None
    target_lost_counter = 0
    MAX_LOST_FRAMES = 10
    x_min, y_min, x_max, y_max = 0, 0, 0, 0

    system_state_names = {
        DeviceState.IDLE: "Idle",
        DeviceState.TRACKING: "Tracking",
        DeviceState.PUMPING: "Pumping",
        DeviceState.RECOVERING: "Recovering"
    }

    while True:
        # 这里省略manage_serial_communication等辅助函数的调用
        with camera_lock:
            if not camera or not camera.isOpened():
                continue
            success, frame = camera.read()
        if not success:
            continue

        # 图像处理
        frame = preprocess(frame)
        ids, scores, bboxes = infer_once(frame)
        results = analysis_results(ids, scores, bboxes)
        frame = draw_results(frame, ids, scores, bboxes)

        frame_height = frame.shape[0]
        frame_width = frame.shape[1]

        in_center_targets = get_targets_in_x_center(results, frame_width, CENTER_X_RATIO)
        if in_center_targets:
            current_target = max(in_center_targets, key=lambda x: x[4])
            in_x_center = True
        else:
            current_target = None
            in_x_center = False

        if current_target:
            target_lost_counter = 0
        elif target_lost_counter > 0:
            target_lost_counter += 1
            if target_lost_counter >= MAX_LOST_FRAMES:
                current_target = None
                target_lost_counter = 0

        if not serial_active:
            device_state["pump"] = 0
            device_state["motor"] = 0
            device_state["direction"] = 0

        current_position = None
        if current_target:
            x_min, y_min, x_max, y_max = current_target[1]
            middle_upper = frame_height * 0.3 - MIDDLE_Y_OFFSET
            middle_lower = frame_height * 0.7 - MIDDLE_Y_OFFSET
            if in_x_center:
                if y_min >= middle_upper and y_max <= middle_lower:
                    current_position = 'middle'
                elif y_min < middle_upper:
                    current_position = 'up'
                elif y_max > middle_lower:
                    current_position = 'down'
                else:
                    current_position = None
            else:
                current_position = 'not_x_center'
            id, bbox, width, height, score = current_target
            cv2.putText(frame, f"Target: ID={id} Conf={score:.2f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if in_x_center:
                cv2.putText(frame, f"Position: {current_position if current_position else 'unknown'}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "未进入左右中心区，不控制", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            current_position = None

        filtered_position = filter_state(current_position) if current_position is not None else None

        if waiting_for_pump_off:
            if time.time() - pump_timer_start >= PUMP_ON_DURATION:
                update_device_state(0, 0, 0, DeviceState.RECOVERING)
                waiting_for_pump_off = False
                pump_timer_start = None

        if filtered_position:
            cv2.putText(frame, f"Filtered: {filtered_position}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if filtered_position == 'up':
                update_device_state(0, 1, 1, DeviceState.TRACKING)
            elif filtered_position == 'down':
                update_device_state(0, 1, 0, DeviceState.TRACKING)
            elif filtered_position == 'middle':
                if device_state["system_state"] == DeviceState.TRACKING:
                    update_device_state(1, 0, device_state["direction"], DeviceState.PUMPING)
                    pump_timer_start = time.time()
                    waiting_for_pump_off = True
            device_state["last_position"] = filtered_position
        else:
            if device_state["system_state"] != DeviceState.IDLE:
                update_device_state(0, 0, 0, DeviceState.IDLE)
            if not current_target:
                cv2.putText(frame, "No target detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            elif current_position == 'not_x_center':
                cv2.putText(frame, "not in x center", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "Position unknown", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        state_text = f"Device: Motor={device_state['motor']} Dir={device_state['direction']} Pump={device_state['pump']}"
        cv2.putText(frame, state_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        state_text = f"System State: {system_state_names.get(device_state['system_state'], 'unknown')}"
        cv2.putText(frame, state_text, (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        if using_second_camera and time.time() - last_photo_time > 5:
            with camera_lock:
                photo_path = os.path.join(img_folder, f'img_{photo_counter}.jpg')
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
def home():
    return render_template('intro.html')

@app.route('/start')
def start():
    return render_template('index4.html')

@app.route('/about')
def about():
    return render_template('about.html')

@app.route('/logout')
def logout():
    session.clear()
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
        using_second_camera = (camera_index == 2)
        if using_second_camera:
            last_photo_time = time.time()
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            return f"❌ 无法打开摄像头 {camera_index}", 500
    return f"✅ 已切换至摄像头 {camera_index}", 200

import atexit
@atexit.register
def cleanup():
    from device import close_serial
    if 'camera' in globals() and camera.isOpened():
        camera.release()
    close_serial()