#ifndef __MOOTOR_H
#define __MOOTOR_H

#include "stm32f4xx.h"
// ��������ˮ�õ�״̬������ʹ��extern�ؼ�������
// �ⲿ��������֪��������Щ�����������ļ����Ѷ���

extern uint8_t pump2_status;
extern uint8_t motor_status;
extern uint8_t motor_direction;
extern uint8_t motor_position;

void Motor_Init(void);                   // �����ʼ��
void Motor_Enable(uint8_t enable);       // ʹ�ܿ���
void Motor_MoveTo(int direction, int enable);  // λ�ÿ���

void WaterPump2_Init(void);  //��ʼ��ˮ�ÿ�������
void WaterPump2_Ctrl(uint8_t state); //����ˮ�ÿ���
#endif
