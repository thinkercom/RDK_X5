#include "mootor.h"
#include "delay.h"


// TODO �� �����������ã�������UART4��ͻ

// TODO �� ��Ҫ����һ�µ����ˮ�õ�״̬��������Щ������extern����һ�£��Һ�ֱ���ڴ�����ֱ�Ӷ�ȡ

//���Ͷ��� �� ˮ���Ƿ���Ҫ����  1���ֽ�   0�ر� 1 ����
//		  �������״̬      1���ֽ�   0�ر�  1����
//		  ������˶�����    1���ֽ�   0�½�  1����
//        �����ʵʱλ��    1���ֽ�   ��ȷ���Ƿ���Ҫ

//֡��Ϊ6���ֽ�  ���е�5���ֽ�ΪУ��λ�����ý�ǰ5λ��������У��
/*���ն��� �� ˮ���Ƿ���  1���ֽ�  0�ر� 1 ����
             ����̨(���)�˶����� 1���ֽ�   0�½� 1����
		     ����̨(���)�˶��Ƿ�ʼ 1���ֽ�  0ֹͣ  1����
*/

// ��������ˮ�õ�״̬����
uint8_t pump2_status = 0;        // ˮ��2״̬ 0�ر� 1��������ʼ��Ϊ�ر�
uint8_t motor_status = 0;        // ���״̬ 0�ر� 1��������ʼ��Ϊ�ر�
uint8_t motor_direction = 0;     // ������� 0�½� 1��������ʼ��Ϊ�½�
uint8_t motor_position = 0;      // ���λ�ã���ʼ��Ϊ0
int current_position = 0;        // ��ǰ����λ�ã���ʼ��Ϊ0
const int MAX_PULSES = 1000;     // ������������Ϊ1000
//�������Ҫ����ֱ�ӰѲ�����ֵ������������
// ���������ջ������ͱ�־λ
//extern uint8_t rx_buffer[6];        // ���ջ���������Ž��յ���6�ֽ�����
//extern uint8_t rx_ready;            // ������ɱ�־

// ����������ã��������ʹ����ͬ������
#define PULSES_TOP     500  // ����λ��������
//#define PULSES_MIDDLE  300  // �м�λ��������
#define PULSES_END     50   // �ײ�λ��������

// ���1���Ŷ��壨PA3/6/7��
#define MOTOR1_EN_PIN    GPIO_Pin_3
#define MOTOR1_EN_PORT   GPIOA
#define MOTOR1_DIR_PIN   GPIO_Pin_6
#define MOTOR1_DIR_PORT  GPIOA
#define MOTOR1_PULSE_PIN GPIO_Pin_7
#define MOTOR1_PULSE_PORT GPIOA

// ���2���Ŷ��壨PC3/5/12��
#define MOTOR2_EN_PIN    GPIO_Pin_3
#define MOTOR2_EN_PORT   GPIOC
#define MOTOR2_DIR_PIN   GPIO_Pin_5
#define MOTOR2_DIR_PORT  GPIOC
#define MOTOR2_PULSE_PIN GPIO_Pin_12
#define MOTOR2_PULSE_PORT GPIOC




// ˮ��2�������Ŷ���
#define WATER_PUMP2_PIN    GPIO_Pin_2
#define WATER_PUMP2_PORT   GPIOA


// �����ʼ��
void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    
    // ����GPIOʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // ���õ��1����
    // ��������
    gpio.GPIO_Pin = MOTOR1_PULSE_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR1_PULSE_PORT, &gpio);
    
    // ��������
    gpio.GPIO_Pin = MOTOR1_DIR_PIN;
    GPIO_Init(MOTOR1_DIR_PORT, &gpio);
    
    // ʹ������
    gpio.GPIO_Pin = MOTOR1_EN_PIN;
    GPIO_Init(MOTOR1_EN_PORT, &gpio);
    
    // ���õ��2����
    // ��������
    gpio.GPIO_Pin = MOTOR2_PULSE_PIN;
    GPIO_Init(MOTOR2_PULSE_PORT, &gpio);
    
    // ��������
    gpio.GPIO_Pin = MOTOR2_DIR_PIN;
    GPIO_Init(MOTOR2_DIR_PORT, &gpio);
    
    // ʹ������
    gpio.GPIO_Pin = MOTOR2_EN_PIN;
    GPIO_Init(MOTOR2_EN_PORT, &gpio);
    
    Motor_Enable(0);  // ��ʼ���õ��
}

// ʹ�ܿ��ƣ�ͬʱ�������������
void Motor_Enable(uint8_t en)
{
    // ����͵�ƽʹ��
    GPIO_WriteBit(MOTOR1_EN_PORT, MOTOR1_EN_PIN, en ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(MOTOR2_EN_PORT, MOTOR2_EN_PIN, en ? Bit_RESET : Bit_SET);
}

void Motor_MoveTo(int direction, int enable)
{
	  // ����Ŀ�������������ݷ����ƶ��̶�������
    const int STEP_SIZE = 100;  // ÿ���ƶ��Ĺ̶�����
    int target_pulses = 0;
    int move_steps;
    int i;
    
    // ���ʹ�ܿ���
    Motor_Enable(enable);
    motor_status = enable;
    
    // ����ǹرյ����ֱ�ӷ���
    if(enable == 0) return;
    switch(direction) {
        case 1:              // ����1�������ƶ�
            target_pulses = current_position + STEP_SIZE;    
            break;
        case 0:              // ����0�������ƶ�
            target_pulses = current_position - STEP_SIZE;    
            break;
        default: return;     // ��Ч����ֱ�ӷ���
    }
    
    // ����Ŀ�����岻�������Χ
    if(target_pulses > MAX_PULSES) {
        target_pulses = MAX_PULSES;
    } else if(target_pulses < 0) {
        target_pulses = 0;
    }

    // ������Ҫ�ƶ��Ĳ���
    move_steps = target_pulses - current_position;
    
    if(move_steps != 0) {
        // �����˶�����
			BitAction dir = (move_steps > 0) ?   Bit_RESET:Bit_SET;
        GPIO_WriteBit(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, dir);
        GPIO_WriteBit(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, dir);
        
        delay_ms(10);
        
        // ����Ŀ������
        for(i=0; i<abs(move_steps); i++) {
            // ����Ƿ�ᳬ�����������
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
            
            // ���µ�ǰλ�ã�ÿ����һ���������һ�Σ�
            current_position += (move_steps > 0) ? 1 : -1;
        }
        
        delay_ms(10);
    }
}




// ��ʼ��ˮ�ÿ�������
void WaterPump2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. ʹ��GPIOAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 2. ����PA2Ϊ�������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        // ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // ����ģʽ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. ��ʼ״̬�ر�ˮ�ã�����͵�ƽ��
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    pump2_status = 0;  // ��ʼ��ˮ��״̬Ϊ�ر�
}

// ����ˮ�ÿ���
void WaterPump2_Ctrl(uint8_t state)
{
    if (state) {
        // ����ˮ�ã�����ߵ�ƽ��
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    } else {
        // �ر�ˮ�ã�����͵�ƽ��
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    }
}
