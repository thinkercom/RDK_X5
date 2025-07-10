# config.py
CENTER_X_RATIO = 0.4  # 左右中心区宽度占总宽度的百分比
MIDDLE_Y_OFFSET = 30  # middle区整体向上平移像素
PUMP_ON_DURATION = 5  # 水泵保持时间（秒）
SERIAL_OFF_DURATION = 3.0  # 串口关闭持续时间（秒）

class DeviceState:
    IDLE = 0            # 空闲状态：未检测到目标
    TRACKING = 1        # 追踪状态：检测到目标并移动
    PUMPING = 2         # 灌溉状态：水泵工作中
    RECOVERING = 3      # 恢复状态：水泵刚结束，底盘运动激活