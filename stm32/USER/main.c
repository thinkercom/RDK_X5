#include "system.h"

// �������ȼ�
#define START_TASK_PRIO    1
#define MOTOR_TASK_PRIO    3   // ��������������ȼ�  ע�����ȼ�
#define PUMP_TASK_PRIO     4   // ˮ�ÿ����������ȼ�

// �����ջ��С
#define START_STK_SIZE     256
#define MOTOR_STK_SIZE     256
#define PUMP_STK_SIZE      256

// ������
TaskHandle_t StartTask_Handler = NULL;
TaskHandle_t MotorTask_Handler = NULL;
TaskHandle_t PumpTask_Handler = NULL;

// ȫ�ֱ�־λ�������ͨ�ţ�
volatile uint8_t motorAtTop = 0;  // ������ﶥ����־
int flag = 1;                     // ˮ�ÿ��Ʊ�־

// ��������
void vTaskMotorControl(void *pvParameters);    // �����������
void vTaskWaterPumpControl(void *pvParameters); // ˮ�ÿ�������
void start_task(void *pvParameters);            // ���������ѷ�װ���̿��ƣ�


//Main function //������
int main(void)
{ 
  systemInit();             // Ӳ����ʼ��
	Motor_Init();             // ��ʼ�������������
  WaterPump2_Init();    
  //Create the start task //������ʼ����
  xTaskCreate((TaskFunction_t )start_task,            //Task function   //������
                            (const char*    )"start_task",          //Task name       //��������
                            (uint16_t       )START_STK_SIZE,        //Task stack size //�����ջ��С
                            (void*          )NULL,                  //Arguments passed to the task function //���ݸ��������Ĳ���
                            (UBaseType_t    )START_TASK_PRIO,       //Task priority   //�������ȼ�
                            (TaskHandle_t*  )&StartTask_Handler);   //Task handle     //������                        
  vTaskStartScheduler();  //Enables task scheduling //�����������   
 // ���´�����ڵ���������ʧ��ʱִ��
    while (1);														
}

 
//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //�����ٽ���
    //Create the task     //��������
       xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);    //Vehicle motion control task //С���˶���������
    xTaskCreate(MPU6050_task,  "MPU6050_task",  MPU6050_STK_SIZE,  NULL, MPU6050_TASK_PRIO,  NULL);    //IMU data read task //IMU���ݶ�ȡ���� 
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED��ʾ����ʾ����
    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);    //LED light flashing task //LED����˸����
    xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);    //Read the PS2 controller task //��ȡPS2�ֱ�����
    xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);    //Usartx3, Usartx1 and CAN send data task //����3������1��CAN������������
    xTaskCreate(vTaskMotorControl,"MotorTask",MOTOR_STK_SIZE,NULL,MOTOR_TASK_PRIO,&MotorTask_Handler);  // ��������������������ƶ�������
    xTaskCreate(vTaskWaterPumpControl,"PumpTask",PUMP_STK_SIZE,NULL,PUMP_TASK_PRIO,&PumpTask_Handler); // ����ˮ�ÿ�������
	  vTaskDelete(StartTask_Handler); //Delete the start task    //ɾ����ʼ����
    taskEXIT_CRITICAL();            //Exit the critical section//�˳��ٽ���
}
// ����������񣨷�����ʵ�֣�
void vTaskMotorControl(void *pvParameters)
{
    while (1) {
			if(motor_status==1){
			Motor_MoveTo(motor_direction,motor_status);
		  vTaskDelay(pdMS_TO_TICKS(2000));  // ��ʱ2�루��������	
			}
		}
}

// ˮ�ÿ������񣨼������״̬��
void vTaskWaterPumpControl(void *pvParameters)
{
    while (1) {
				if (pump2_status == 1) {
            WaterPump2_Ctrl(1);  // ����ˮ��11
        }
				else if(pump2_status == 0){
            WaterPump2_Ctrl(0);  // �ر�ˮ��1
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // ����ʱ������CPUռ��
    }
}