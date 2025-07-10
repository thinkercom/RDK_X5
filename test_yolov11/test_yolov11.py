from ultralytics import YOLO
import cv2

# 加载 yolov11n.pt 模型需要换路径(可以换成自己的路径)
model = YOLO('/home/sunrise/Project20250627/test_yolov11/yolov11n.pt')

# 推理图片（返回 DetectResult 对象列表）
results = model('/home/sunrise/Project20250627/test_yolov11/demo3.jpg',conf=0.15)  

# 可视化结果
# results[0].show()       # 弹窗显示
results[0].save(filename='/home/sunrise/Project20250627/test_yolov11/render3.jpg')  # 保存到本地
