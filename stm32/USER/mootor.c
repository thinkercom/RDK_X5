#include "mootor.h"
#include "delay.h"


// TODO ： 更改引脚配置，避免与UART4冲突

// TODO ： 需要定义一下电机和水泵的状态参数，这些变量用extern修饰一下，我好直接在串口里直接读取

//发送定义 ： 水泵是否需要开启  1个字节   0关闭 1 开启
//		  电机开启状态      1个字节   0关闭  1开启
//		  电机的运动方向    1个字节   0下降  1上升
//        电机的实时位置    1个字节   待确定是否需要

//帧长为6个字节  其中第5个字节为校验位，采用将前5位做异或进行校验
/*接收定义 ： 水泵是否开启  1个字节  0关闭 1 开启
             升降台(电机)运动方向 1个字节   0下降 1上升
		     升降台(电机)运动是否开始 1个字节  0停止  1开启
*/

// 定义电机和水泵的状态变量
uint8_t pump2_status = 0;        // 水泵2状态 0关闭 1开启，初始化为关闭
uint8_t motor_status = 0;        // 电机状态 0关闭 1开启，初始化为关闭
uint8_t motor_direction = 0;     // 电机方向 0下降 1上升，初始化为下降
uint8_t motor_position = 0;      // 电机位置，初始化为0
int current_position = 0;        // 当前脉冲位置，初始化为0
const int MAX_PULSES = 1000;     // 脉冲总数限制为1000
//这个不必要，我直接把参数赋值赋过来就行了
// 新增：接收缓冲区和标志位
//extern uint8_t rx_buffer[6];        // 接收缓冲区，存放接收到的6字节数据
//extern uint8_t rx_ready;            // 接收完成标志

// 电机参数配置（两个电机使用相同参数）
#define PULSES_TOP     500  // 顶部位置脉冲数
//#define PULSES_MIDDLE  300  // 中间位置脉冲数
#define PULSES_END     50   // 底部位置脉冲数

// 电机1引脚定义（PA3/6/7）
#define MOTOR1_EN_PIN    GPIO_Pin_3
#define MOTOR1_EN_PORT   GPIOA
#define MOTOR1_DIR_PIN   GPIO_Pin_6
#define MOTOR1_DIR_PORT  GPIOA
#define MOTOR1_PULSE_PIN GPIO_Pin_7
#define MOTOR1_PULSE_PORT GPIOA

// 电机2引脚定义（PC3/5/12）
#define MOTOR2_EN_PIN    GPIO_Pin_3
#define MOTOR2_EN_PORT   GPIOC
#define MOTOR2_DIR_PIN   GPIO_Pin_5
#define MOTOR2_DIR_PORT  GPIOC
#define MOTOR2_PULSE_PIN GPIO_Pin_12
#define MOTOR2_PULSE_PORT GPIOC




// 水泵2控制引脚定义
#define WATER_PUMP2_PIN    GPIO_Pin_2
#define WATER_PUMP2_PORT   GPIOA


// 电机初始化
void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    
    // 开启GPIO时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 配置电机1引脚
    // 脉冲引脚
    gpio.GPIO_Pin = MOTOR1_PULSE_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR1_PULSE_PORT, &gpio);
    
    // 方向引脚
    gpio.GPIO_Pin = MOTOR1_DIR_PIN;
    GPIO_Init(MOTOR1_DIR_PORT, &gpio);
    
    // 使能引脚
    gpio.GPIO_Pin = MOTOR1_EN_PIN;
    GPIO_Init(MOTOR1_EN_PORT, &gpio);
    
    // 配置电机2引脚
    // 脉冲引脚
    gpio.GPIO_Pin = MOTOR2_PULSE_PIN;
    GPIO_Init(MOTOR2_PULSE_PORT, &gpio);
    
    // 方向引脚
    gpio.GPIO_Pin = MOTOR2_DIR_PIN;
    GPIO_Init(MOTOR2_DIR_PORT, &gpio);
    
    // 使能引脚
    gpio.GPIO_Pin = MOTOR2_EN_PIN;
    GPIO_Init(MOTOR2_EN_PORT, &gpio);
    
    Motor_Enable(0);  // 初始禁用电机
}

// 使能控制（同时控制两个电机）
void Motor_Enable(uint8_t en)
{
    // 假设低电平使能
    GPIO_WriteBit(MOTOR1_EN_PORT, MOTOR1_EN_PIN, en ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(MOTOR2_EN_PORT, MOTOR2_EN_PIN, en ? Bit_RESET : Bit_SET);
}

void Motor_MoveTo(int direction, int enable)
{
	  // 设置目标脉冲数（根据方向移动固定步数）
    const int STEP_SIZE = 100;  // 每次移动的固定步数
    int target_pulses = 0;
    int move_steps;
    int i;
    
    // 电机使能控制
    Motor_Enable(enable);
    motor_status = enable;
    
    // 如果是关闭电机，直接返回
    if(enable == 0) return;
    switch(direction) {
        case 1:              // 输入1：向上移动
            target_pulses = current_position + STEP_SIZE;    
            break;
        case 0:              // 输入0：向下移动
            target_pulses = current_position - STEP_SIZE;    
            break;
        default: return;     // 无效输入直接返回
    }
    
    // 限制目标脉冲不超过最大范围
    if(target_pulses > MAX_PULSES) {
        target_pulses = MAX_PULSES;
    } else if(target_pulses < 0) {
        target_pulses = 0;
    }

    // 计算需要移动的步数
    move_steps = target_pulses - current_position;
    
    if(move_steps != 0) {
        // 设置运动方向
			BitAction dir = (move_steps > 0) ?   Bit_RESET:Bit_SET;
        GPIO_WriteBit(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, dir);
        GPIO_WriteBit(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, dir);
        
        delay_ms(10);
        
        // 发送目标脉冲
        for(i=0; i<abs(move_steps); i++) {
            // 检查是否会超出最大脉冲数
            if((move_steps > 0 && current_position >= MAX_PULSES) || 
               (move_steps < 0 && current_position <= 0)) {
                break;
            }
            
            GPIO_SetBits(MOTOR1_PULSE_PORT, MOTOR1_PULSE_PIN);
            GPIO_SetBits(MOTOR2_PULSE_PORT, MOTOR2_PULSE_PIN);
            delay_us(900);
            GPIO_ResetBits(MOTOR1_PULSE_PORT, MOTOR1_PULSE_PIN);
            GPIO_ResetBits(MOTOR2_PULSE_PORT, MOTOR2_PULSE_PIN);
            delay_us(900);
            
            // 更新当前位置（每发送一个脉冲更新一次）
            current_position += (move_steps > 0) ? 1 : -1;
        }
        
        delay_ms(10);
    }
}




// 初始化水泵控制引脚
void WaterPump2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能GPIOA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 2. 配置PA2为推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 高速模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // 无上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 初始状态关闭水泵（输出低电平）
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    pump2_status = 0;  // 初始化水泵状态为关闭
}

// 控制水泵开关
void WaterPump2_Ctrl(uint8_t state)
{
    if (state) {
        // 开启水泵（输出高电平）
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    } else {
        // 关闭水泵（输出低电平）
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    }
}
