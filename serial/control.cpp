#include "WzSerialportPlus.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <mutex>

// 数据帧格式定义
const uint8_t FRAME_HEADER = 0X7B;
const uint8_t FRAME_TAIL = 0x7D;

// 设备状态定义
enum PumpStatus {
    PUMP_OFF = 0x00,
    PUMP_ON = 0x01,
    
};

enum MotorStatus {
    MOTOR_STOPPED = 0x00,
    MOTOR_RUNNING = 0x01
};

enum MotorDirection {
    DIRECTION_DOWN = 0x00,    // 上升
    DIRECTION_UP = 0x01    // 下降
};

class SerialComm {
public:
    SerialComm(const std::string& port )
        : serialPort(port, 115200, 1, 8, 'n')
    {
        serialPort.setReceiveCalback([this](char* data, int length) {
            this->handleReceive(data, length);
        });
    }

    bool start() {
        if (!serialPort.open()) {
            std::cerr << "无法打开串口" << std::endl;
            return false;
        }
        
        // 启动发送线程
        sendThread = std::thread(&SerialComm::sendLoop, this);
        std::cout << "串口通信已启动" << std::endl;
        return true;
    }

    void stop() {
        running = false;
        if (sendThread.joinable()) {
            sendThread.join();
        }
        serialPort.close();
        std::cout << "串口通信已停止" << std::endl;
    }

    // 更新设备状态 (可在外部调用)
    void updateStatus(PumpStatus pump, MotorStatus motor, MotorDirection dir) {
        std::lock_guard<std::mutex> lock(statusMutex);
        pumpStatus = pump;
        motorStatus = motor;
        motorDirection = dir;
        statusUpdated = true;
    }

private:
    void sendLoop() {
        // 等待下位机初始化完成 (开机10秒)
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        while (running) {
            // 检查是否有状态更新
            bool shouldSend = false;
            {
                std::lock_guard<std::mutex> lock(statusMutex);
                shouldSend = statusUpdated;
                statusUpdated = false;
            }
            
            if (shouldSend) {
                sendStatusFrame();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void sendStatusFrame() {
        // 获取当前状态
        uint8_t pump, motor, dir;
        {
            std::lock_guard<std::mutex> lock(statusMutex);
            pump = pumpStatus;
            motor = motorStatus;
            dir = motorDirection;
        }
        
        // 构造数据帧
        uint8_t frame[6] = {0};
        frame[0] = FRAME_HEADER;
        frame[1] = pump;
        frame[2] = motor;
        frame[3] = dir;
        frame[4] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3]; // 计算校验位
        frame[5] = FRAME_TAIL;
        
        // 发送数据
        int sent = serialPort.send(reinterpret_cast<char*>(frame), sizeof(frame));
        if (sent != sizeof(frame)) {
            std::cerr << "发送失败，实际发送: " << sent << "/" << sizeof(frame) << " 字节" << std::endl;
        } else {
            std::cout << "已发送状态帧: "
                      << "Pump=" << static_cast<int>(pump)
                      << ", Motor=" << static_cast<int>(motor)
                      << ", Dir=" << static_cast<int>(dir) << std::endl;
        }
    }

    void handleReceive(char* data, int length) {
        // 简化的接收处理 - 仅打印接收数据
        std::cout << "接收到 " << length << " 字节数据: ";
        for (int i = 0; i < length; ++i) {
            printf("%02X ", static_cast<uint8_t>(data[i]));
        }
        std::cout << std::endl;
    }

    WzSerialportPlus serialPort;
    std::thread sendThread;
    std::mutex statusMutex;
    bool running = true;
    bool statusUpdated = false;
    PumpStatus pumpStatus = PUMP_OFF;
    MotorStatus motorStatus = MOTOR_STOPPED;
    MotorDirection motorDirection = DIRECTION_DOWN;
};


// test 测试串口是否发送成功
// 模拟10次状态变化
int main() {

    //初始化，创建串口实例，自动创建线程发送数据
    SerialComm comm("/dev/ttyS1");
    //打开串口
    if (!comm.start()) {
        return 1;
    }
    
    // 模拟状态变化
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 随机生成状态变化
        PumpStatus pump = (i % 2 == 0) ? PUMP_OFF : PUMP_ON;
        
        MotorStatus motor = (i % 4 == 0) ? MOTOR_STOPPED : MOTOR_RUNNING;
        
        MotorDirection dir = (i % 2 == 0) ? DIRECTION_DOWN : DIRECTION_UP;
        
        //调用该函数更新状态
        comm.updateStatus(pump, motor, dir);
    }
    //关闭串口 写在关闭整个程序的地方
    comm.stop();
    return 0;
}

// 封装函数，用于 Python 调用
extern "C" {
    SerialComm* SerialComm_new(const char* port) {
        return new SerialComm(std::string(port));
    }

    int SerialComm_start(SerialComm* obj) {
        return obj->start() ? 0 : 1;
    }

    void SerialComm_stop(SerialComm* obj) {
        obj->stop();
    }

    void SerialComm_updateStatus(SerialComm* obj, uint8_t pump, uint8_t motor, uint8_t dir) {
        obj->updateStatus(static_cast<PumpStatus>(pump), static_cast<MotorStatus>(motor), static_cast<MotorDirection>(dir));
    }
}