#ifndef __MOOTOR_H
#define __MOOTOR_H

#include "stm32f4xx.h"
// 定义电机和水泵的状态参数，使用extern关键字修饰
// 外部声明：告知编译器这些变量在其他文件中已定义

extern uint8_t pump2_status;
extern uint8_t motor_status;
extern uint8_t motor_direction;
extern uint8_t motor_position;

void Motor_Init(void);                   // 电机初始化
void Motor_Enable(uint8_t enable);       // 使能控制
void Motor_MoveTo(int direction, int enable);  // 位置控制

void WaterPump2_Init(void);  //初始化水泵控制引脚
void WaterPump2_Ctrl(uint8_t state); //控制水泵开关
#endif
