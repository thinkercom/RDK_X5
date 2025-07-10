#include "system.h"

// 任务优先级
#define START_TASK_PRIO    1
#define MOTOR_TASK_PRIO    3   // 电机控制任务优先级  注意优先级
#define PUMP_TASK_PRIO     4   // 水泵控制任务优先级

// 任务堆栈大小
#define START_STK_SIZE     256
#define MOTOR_STK_SIZE     256
#define PUMP_STK_SIZE      256

// 任务句柄
TaskHandle_t StartTask_Handler = NULL;
TaskHandle_t MotorTask_Handler = NULL;
TaskHandle_t PumpTask_Handler = NULL;

// 全局标志位（任务间通信）
volatile uint8_t motorAtTop = 0;  // 电机到达顶部标志
int flag = 1;                     // 水泵控制标志

// 函数声明
void vTaskMotorControl(void *pvParameters);    // 电机控制任务
void vTaskWaterPumpControl(void *pvParameters); // 水泵控制任务
void start_task(void *pvParameters);            // 启动任务（已封装底盘控制）


//Main function //主函数
int main(void)
{ 
  systemInit();             // 硬件初始化
	Motor_Init();             // 初始化电机控制引脚
  WaterPump2_Init();    
  //Create the start task //创建开始任务
  xTaskCreate((TaskFunction_t )start_task,            //Task function   //任务函数
                            (const char*    )"start_task",          //Task name       //任务名称
                            (uint16_t       )START_STK_SIZE,        //Task stack size //任务堆栈大小
                            (void*          )NULL,                  //Arguments passed to the task function //传递给任务函数的参数
                            (UBaseType_t    )START_TASK_PRIO,       //Task priority   //任务优先级
                            (TaskHandle_t*  )&StartTask_Handler);   //Task handle     //任务句柄                        
  vTaskStartScheduler();  //Enables task scheduling //开启任务调度   
 // 以下代码仅在调度器启动失败时执行
    while (1);														
}

 
//Start task task function //开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //进入临界区
    //Create the task     //创建任务
       xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);    //Vehicle motion control task //小车运动控制任务
    xTaskCreate(MPU6050_task,  "MPU6050_task",  MPU6050_STK_SIZE,  NULL, MPU6050_TASK_PRIO,  NULL);    //IMU data read task //IMU数据读取任务 
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED显示屏显示任务
    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);    //LED light flashing task //LED灯闪烁任务
    xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);    //Read the PS2 controller task //读取PS2手柄任务
    xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);    //Usartx3, Usartx1 and CAN send data task //串口3、串口1、CAN发送数据任务
    xTaskCreate(vTaskMotorControl,"MotorTask",MOTOR_STK_SIZE,NULL,MOTOR_TASK_PRIO,&MotorTask_Handler);  // 创建电机控制任务（上下移动机构）
    xTaskCreate(vTaskWaterPumpControl,"PumpTask",PUMP_STK_SIZE,NULL,PUMP_TASK_PRIO,&PumpTask_Handler); // 创建水泵控制任务
	  vTaskDelete(StartTask_Handler); //Delete the start task    //删除开始任务
    taskEXIT_CRITICAL();            //Exit the critical section//退出临界区
}
// 电机控制任务（非阻塞实现）
void vTaskMotorControl(void *pvParameters)
{
    while (1) {
			if(motor_status==1){
			Motor_MoveTo(motor_direction,motor_status);
		  vTaskDelay(pdMS_TO_TICKS(2000));  // 延时2秒（非阻塞）	
			}
		}
}

// 水泵控制任务（监听电机状态）
void vTaskWaterPumpControl(void *pvParameters)
{
    while (1) {
				if (pump2_status == 1) {
            WaterPump2_Ctrl(1);  // 开启水泵11
        }
				else if(pump2_status == 0){
            WaterPump2_Ctrl(0);  // 关闭水泵1
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 短延时，降低CPU占用
    }
}