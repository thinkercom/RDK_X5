#include "balance.h"

int Time_count=0; //Time variable //计时变量
int protect_count=0; //Time variable //计时变量
uint32_t stopkey_count=0; //Time variable //计时变量
bool stopkey_pressed = false;
int forcestop = 0;
uint32_t parking_time;

// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 
extern Robot_Parament_InitTypeDef	Robot_Parament; 
short test_num;

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
            //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
      {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
		}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		    MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		    MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	        MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  uint32_t lastWakeTime = getSysTickCnt();
    int i =0;
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			if((pump2_status||motor_status)==0)
			{
			if(i++ == 200){
			    i= 0;
	            if(RosCmdUpdate == 0)
	            {
	               Move_X=0;
				   Move_Y=0;
				   Move_Z=0;
				}
				RosCmdUpdate = 0;
		   	}
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();   
			if(Servo_init_angle_adjust == 1) Servo_init_angle_adjust_function(); //进入舵机初始位置微调模式
			else Servo_angle_adjust_function();
			if(Check==0) //If self-check mode is not enabled //如果没有启动自检模式
			{
				if (APP_ON_Flag)           Get_RC();         //Handle the APP remote commands //处理APP遥控命令
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令
				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				//Click the user button to update the gyroscope zero
				//单击用户按键更新陀螺仪零点
				Key(); 
				Drive_Robot_Arm();//机械臂控制
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position
                //and the software failure flag is 0
				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
				if(Turn_Off(Voltage)==0) 
				 { 			
                     //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速

					 if(force_stop)MOTOR_A.Target = MOTOR_B.Target=MOTOR_C.Target=MOTOR_D.Target =0;
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

                     #if 0
					 MOTOR_A.Motor_Pwm= MOTOR_A.Target*20000;
					 MOTOR_B.Motor_Pwm= MOTOR_B.Target*20000;
					 MOTOR_C.Motor_Pwm= MOTOR_C.Target*20000;
					 MOTOR_D.Motor_Pwm= MOTOR_D.Target*20000;
                     #endif

                     if(MOTOR_A.Target == 0 && MOTOR_B.Target == 0 && MOTOR_C.Target == 0 && MOTOR_D.Target == 0 ) parking_time++;
                     else parking_time = 0;
					 if(parking_time < 500){
                         if(MOTOR_A.Target == 0 && abs(MOTOR_A.Encoder*1000) < 20 )   MOTOR_A.Motor_Pwm = 0;					
                         if(MOTOR_B.Target == 0 && abs(MOTOR_B.Encoder*1000) < 20 )   MOTOR_B.Motor_Pwm = 0;	
                         if(MOTOR_C.Target == 0 && abs(MOTOR_C.Encoder*1000) < 20 )   MOTOR_C.Motor_Pwm = 0;	
                         if(MOTOR_D.Target == 0 && abs(MOTOR_D.Encoder*1000) < 20 )	  MOTOR_D.Motor_Pwm = 0;
					   }
					 Limit_Pwm(15960);
					 #if 0
				     if(stopkey_pressed == false && forcestop == false && force_stop == false)
				     	{
				     	 if((abs(MOTOR_A.Motor_Pwm) == 15960 && abs(MOTOR_A.Encoder*1000) < 10 )&&(abs(MOTOR_B.Motor_Pwm) == 15960 && abs(MOTOR_B.Encoder*1000)< 10)) stopkey_count++;  
					     else if(stopkey_count > 0)stopkey_count--;  
					     if(stopkey_count > 50)stopkey_pressed = true;
				     	}

					 else 
					 {
                         if( abs(MOTOR_A.Encoder*1000) > 10 || abs(MOTOR_B.Encoder*1000) > 10) stopkey_pressed = false,stopkey_count =0;
					 }
					 #endif
					 if(stopkey_pressed == false)
					 {
	                     if((abs(MOTOR_A.Motor_Pwm) > 15959.0 && (MOTOR_A.Target*MOTOR_A.Encoder <= 0))||(abs(MOTOR_B.Motor_Pwm) > 15959.0 && (MOTOR_B.Target*MOTOR_B.Encoder <= 0)) ||
						 	(abs(MOTOR_C.Motor_Pwm) > 15959.0 && (MOTOR_C.Target*MOTOR_C.Encoder <= 0))||(abs(MOTOR_D.Motor_Pwm) > 15959.0 && (MOTOR_D.Target*MOTOR_D.Encoder <= 0)))
	
	                         protect_count++; 
	                     else  protect_count=0;
	                     if(protect_count > 100) forcestop = 1;
	                     else if( MOTOR_A.Target == 0 && MOTOR_B.Target == 0 && MOTOR_C.Target == 0 && MOTOR_D.Target == 0) forcestop = 0;
					 }
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
					 switch(Car_Mode)
					 {
						case Mec_Car:       Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0 );  break;    //四驱车	
						case Omni_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
						case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //阿克曼小车
						case Diff_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm,  0, 0, 0); break; //两轮差速小车
					    case FourWheel_Car: Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0    );   break; //四驱车
						case Tank_Car:      Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, 0, 0, 0);   break; 	
					 }

					 
				 }

				  
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
				 else	Set_Pwm(0,0,0,0,0); 
			 }	
		 }
			else Set_Pwm(0,0,0,0,0); //水泵或滑台移动时候，不允许控制小车进行运动，PWM值设置为0
				
	 }			
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制force_stop || forcestop
	if(force_stop || forcestop ) motor_a=motor_b=motor_c= motor_d =0;
    #if M1_REVERSE
    motor_a = -motor_a;
    #endif
    #if M2_REVERSE
    motor_b = -motor_b;
    #endif
    #if M3_REVERSE
    motor_c = -motor_c;
    #endif
    #if M4_REVERSE
    motor_d = -motor_d;
    #endif
	#if HIGH_POWER_DRIVER_V2
		if(motor_a<0)	    AIN2=0,	   PWMA1=0;
		else if(motor_a >0) AIN2=0,    PWMA1=16800;
		else AIN2=1;
		PWMA2= abs(motor_a);
	
		if(motor_b<0)	    BIN2=0,	   PWMB2=0;
		else if(motor_b>0)	BIN2=0,    PWMB2=16800;
		else BIN2=1;
		PWMB1= abs(motor_b);
	
		if(motor_c>0)	    CIN2=0,	   PWMC2=0;
		else if(motor_c<0)	CIN2=0,    PWMC2=16800;
		else CIN2=1;
		PWMC1= abs(motor_c);
	
		if(motor_d>0)		DIN2=0,    PWMD2=0;
		else if(motor_d<0)	DIN2=0,    PWMD2=16800;
		else DIN2=1;
		PWMD1= abs(motor_d);
    #else
		
		if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
		else 	                PWMA2=16799,PWMA1=16799-motor_a;

		//Forward and reverse control of motor
		//电机正反转控制	
		if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
		else 	                PWMB2=16799,PWMB1=16799-motor_b;
	
		//Forward and reverse control of motor
		//电机正反转控制	
		if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
		else 	                PWMC2=16799,PWMC1=16799-motor_c;

		//Forward and reverse control of motor
		//电机正反转控制
		if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
		else 	                PWMD2=16799,PWMD1=16799-motor_d;		
#endif

	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
uint8_t Turn_Off( int voltage)
{
    uint8_t temp;
	if(voltage<10||EN==0||Flag_Stop==1)
	{	                                                
		temp=1;      
		PWMA1=0;PWMA2=0;
		PWMB1=0;PWMB2=0;		
		PWMC1=0;PWMC1=0;	
		PWMD1=0;PWMD2=0;					
    }
	else
	temp=0;
	return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{ 		   
	  uint32_t temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 if(Target == 0 && Bias == 0)Pwm = 0; 

	 if(stopkey_pressed){
        if(Pwm > 3000) Pwm = 3000;
		else if(Pwm < -3000) Pwm = -3000;

	 }
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 if(Target == 0 && Bias == 0)Pwm = 0; 
	 if(stopkey_pressed){
        if(Pwm > 3000) Pwm = 3000;
		else if(Pwm < -3000) Pwm = -3000;

	 }
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 if(Target == 0 && Bias == 0)Pwm = 0; 
	 if(stopkey_pressed){
        if(Pwm > 3000) Pwm = 3000;
		else if(Pwm < -3000) Pwm = -3000;

	 }
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 if(Target == 0 && Bias == 0)Pwm = 0; 
	 if(stopkey_pressed){
        if(Pwm > 3000) Pwm = 3000;
		else if(Pwm < -3000) Pwm = -3000;

	 }
	 return Pwm; 
}
#if 1
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID2 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID3 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID4 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID5 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID6 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
#endif




/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	uint8_t Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 3:      Move_X=0;      		 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	 Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		             Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;     break;
			case 3:      Move_X=0;      		 Move_Z=-PI/2;     break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	uint8_t tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/


void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		
     #if SHOW_SPEED == 1
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);
     #else
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);
        OriginalEncoder.E += OriginalEncoder.A;
        OriginalEncoder.F += OriginalEncoder.B;
        OriginalEncoder.G += OriginalEncoder.C;
		OriginalEncoder.H += OriginalEncoder.D;
        #endif
	    switch(Car_Mode)
        {
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break;    //四驱车	
			case Omni_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break; //全向轮小车
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break;//阿克曼小车
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B;                                                                       break; //两轮差速小车
		    case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break; //四驱车
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B;                                                                       break; 	
		}
        #if HALL1_REVERSE
		Encoder_A_pr= -Encoder_A_pr; 
		#endif
		#if HALL2_REVERSE
		Encoder_B_pr= -Encoder_B_pr;
		#endif
		#if HALL3_REVERSE
		Encoder_C_pr= -Encoder_C_pr; 
		#endif
		#if HALL4_REVERSE
		Encoder_D_pr= -Encoder_D_pr;	
		#endif

		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


#if 1
/**************************************************************************
函数功能：机械臂关节角度转对应pwm值函数
入口参数：机械臂目标关节角度
返回  值：关节角度对应的pwm值
**************************************************************************/
int SERVO_PWM_VALUE(int id,float angle)
{
    #if 0
    float Ratio;
	int K=1000;
    if(id <= 2) Ratio=0.64;
	else  Ratio=0.424;
	int pwm_value;
	pwm_value=500+(angle*K*Ratio); //舵机目标
    return pwm_value;
	#else
    int K=1000;
	float Ratio=0.64;
	int pwm_value;
	pwm_value=(SERVO_INIT-angle*K*Ratio); //舵机目标
    return pwm_value;
	#endif
}
/**************************************************************************
函数功能：机械臂关节角度限位函数
入口参数：无
返回  值：无
**************************************************************************/
void moveit_angle_limit(void)
{ 
    Moveit_Angle1=target_limit_float(Moveit_Angle1,-1.57,1.57);
	Moveit_Angle2=target_limit_float(Moveit_Angle2,-1.57,1.57);
	Moveit_Angle3=target_limit_float(Moveit_Angle3,-1.57,1.57);
	Moveit_Angle4=target_limit_float(Moveit_Angle4,-0.3,1.57);
	Moveit_Angle5=target_limit_float(Moveit_Angle5,-1.57,1.57);
	Moveit_Angle6=target_limit_float(Moveit_Angle6,-0.7,0.7);  //夹爪可运动范围较小
	
}
/**************************************************************************
函数功能：机械臂关节PWM值限位函数
入口参数：无
返回  值：无
**************************************************************************/
void moveit_pwm_limit(void)
{ 
	Moveit_PWM1=target_limit_int(Moveit_PWM1,1000,1800);
	Moveit_PWM2=target_limit_int(Moveit_PWM2,400,2600);
	Moveit_PWM3=target_limit_int(Moveit_PWM3,400,2600);
	Moveit_PWM4=target_limit_int(Moveit_PWM4,400,2600);
	Moveit_PWM5=target_limit_int(Moveit_PWM5,1100,1900);
	Moveit_PWM6=target_limit_int(Moveit_PWM6,900,2100);  
}

/**************************************************************************
函数功能：机械臂关节驱动部分代码，使用PID位置开环控制
入口参数：无
返回  值：无
**************************************************************************/


int pwm1,pwm2,pwm3,pwm4;
int pwm1_=1500,pwm2_=1500,pwm3_=1500,pwm4_=1500;

int cnt = 0;
uint32_t now;
uint8_t ServoSendCmd = 1;
int Servo1Status = 1,Servo2Status = 1,Servo3Status = 1;
void Drive_Robot_Arm(void)
{     
      moveit_angle_limit();		 //关节角度做限幅
      Moveit_PWM1=  SERVO_PWM_VALUE(1,Moveit_Angle1)+Moveit_Angle1_init; //输入目标弧度，输出舵机目标PWM值
      Moveit_PWM2 = SERVO_PWM_VALUE(2,Moveit_Angle2)+Moveit_Angle2_init;
      Moveit_PWM3 = SERVO_PWM_VALUE(3,Moveit_Angle3)+Moveit_Angle3_init;
      Moveit_PWM4 = SERVO_PWM_VALUE(4,Moveit_Angle4)+Moveit_Angle4_init;
      Moveit_PWM5 = SERVO_PWM_VALUE(5,Moveit_Angle5)+Moveit_Angle5_init;
      Moveit_PWM6 = SERVO_PWM_VALUE(6,Moveit_Angle6)+Moveit_Angle6_init;
     // moveit_pwm_limit();

	  #if 0
      Velocity1=Position_PID1(Position1,Moveit_PWM1);
      Velocity2=Position_PID2(Position2,Moveit_PWM2);
      Velocity3=Position_PID3(Position3,Moveit_PWM3);
      Velocity4=Position_PID4(Position4,Moveit_PWM4);
      Velocity5=Position_PID5(Position5,Moveit_PWM5);
      Velocity6=Position_PID6(Position6,Moveit_PWM6);
	  

      Position1+=Velocity1;		   //速度的积分，得到舵机的位置
      Position2+=Velocity2;
      Position3+=Velocity3;
      Position4+=Velocity4;
      Position5+=Velocity5;	
      Position6+=Velocity6;		
      #else
      Position1= Moveit_PWM1;
	  Position2= Moveit_PWM2;
	  Position3= Moveit_PWM3;
	  Position4= Moveit_PWM4;
	  #endif
	  Servo_PWM1 =Position1;
      Servo_PWM2 =Position2;
	  Servo_PWM3 =Position3;
      Servo_PWM4 =Position4;
      pwm1 = Position1;
      pwm2 = Position2;
      pwm3 = Position3;
      pwm4 = Position4;
	  if(cnt++ == 10)
	  {
         static int needtoreq = 0;
         cnt = 0;
         if(getSysTickCnt() > 5000){
            if(pwm1_ != pwm1)needtoreq = 1, now = getSysTickCnt(), BusServoCtrl(1, pwm1,100),pwm1_ = pwm1;
            if(pwm2_ != pwm2)needtoreq = 1, now = getSysTickCnt(), BusServoCtrl(2, pwm2,100),pwm2_ = pwm2;
            if(pwm3_ != pwm3)needtoreq = 1, now = getSysTickCnt(), BusServoCtrl(3, pwm3,100),pwm3_ = pwm3;
			if(pwm4_ != pwm4)needtoreq = 1, now = getSysTickCnt(), BusServoCtrl(4, pwm4,100),pwm4_ = pwm4;
            if(needtoreq == 1 && getSysTickCnt() - now >= 500) needtoreq = 0,ServoSendCmd = 1;

			#if 0
            if(abs(pwm1 -S1Pos) > 50  && getSysTickCnt()- getservo1time > 10000) UART5_PutStr("#001PULK!"),Servo1Status = 0;
            else if( Servo1Status == 0){};
			
			if(abs(pwm2 -S2Pos) > 50  && getSysTickCnt()- getservo2time > 10000) UART5_PutStr("#002PULK!"),Servo2Status = 0;
			else if( Servo2Status == 0){};
			if(abs(pwm3 -S3Pos) > 50  && getSysTickCnt()- getservo3time > 10000) UART5_PutStr("#003PULK!"),Servo3Status = 0;
			else if( Servo3Status == 0){};
			#endif
         }
	  }
	  
}
#endif

