# device.py
import ctypes
import logging
from config import DeviceState

logging.basicConfig(
    level = logging.DEBUG,
    format = '[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S')
logger = logging.getLogger("RDK_YOLO")

lib = ctypes.CDLL('/home/sunrise/Project20250627/serial/libserialcomm.so')
lib.SerialComm_new.argtypes = [ctypes.c_char_p]
lib.SerialComm_new.restype = ctypes.c_void_p
lib.SerialComm_start.argtypes = [ctypes.c_void_p]
lib.SerialComm_start.restype = ctypes.c_int
lib.SerialComm_stop.argtypes = [ctypes.c_void_p]
lib.SerialComm_updateStatus.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8]

serial_port = b'/dev/ttyS1'
comm = lib.SerialComm_new(serial_port)
if lib.SerialComm_start(comm) != 0:
    logger.error("Failed to start serial communication.")
    exit(1)

device_state = {
    "pump": 0,
    "motor": 0,
    "direction": 0,
    "last_position": None,
    "system_state": DeviceState.IDLE
}

def update_device_state(pump, motor, direction, system_state):
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
            "system_state": system_state
        }
        logger.info(f"Device State Updated - PUMP:{pump} MOTOR:{motor} DIR:{direction} STATE:{system_state}")

def close_serial():
    lib.SerialComm_stop(comm)