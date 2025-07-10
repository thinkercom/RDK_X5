#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)

extern short test_num;
extern int robot_mode_check_flag;
void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
uint8_t Turn_Off( int voltage);
uint32_t myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
float Position_PID1 (float Encoder,float Target);
float Position_PID2 (float Encoder,float Target);
float Position_PID3 (float Encoder,float Target);
float Position_PID4 (float Encoder,float Target);
float Position_PID5 (float Encoder,float Target);
float Position_PID6 (float Encoder,float Target);

void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
void PS2_control(void);
float float_abs(float insert);
void robot_mode_check(void);
void Drive_Robot_Arm(void);
int SERVO_PWM_VALUE(int id,float angle);
void moveit_angle_limit(void);
void moveit_pwm_limit(void);
extern uint8_t ServoSendCmd;

#endif  

