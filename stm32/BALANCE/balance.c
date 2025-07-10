#include "balance.h"

int Time_count=0; //Time variable //��ʱ����
int protect_count=0; //Time variable //��ʱ����
uint32_t stopkey_count=0; //Time variable //��ʱ����
bool stopkey_pressed = false;
int forcestop = 0;
uint32_t parking_time;

// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ�������־λ
int robot_mode_check_flag=0; 
extern Robot_Parament_InitTypeDef	Robot_Parament; 
short test_num;

Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //ȫ���ƶ�С���ſ����ٶ�ƽ������
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������
  
            //Get the smoothed data 
			//��ȡƽ������������			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //�����ķ��С��
	  if (Car_Mode==Mec_Car) 
      {
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//ȫ����С��
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //û��ʹ�õ�
		}
		
		//Ackermann structure car
		//������С��
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //������С��ר����ر���
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//���ڰ�����С��Vz������ǰ��ת��Ƕ�
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //�˶�ѧ���
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
			//���PWMֵ���������ǰ��ת��Ƕ�
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //���PWMֵ�޷�
		}
		
		//Differential car
		//����С��
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}
		
		//FourWheel car
		//������
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
					
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		
		//Tank Car
		//�Ĵ���
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		    MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		    MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	        MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  uint32_t lastWakeTime = getSysTickCnt();
    int i =0;
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//��������100Hz��Ƶ�����У�10ms����һ�Σ�
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
			//ʱ�������30�������Ҫ
			if(Time_count<3000)Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
			Get_Velocity_Form_Encoder();   
			if(Servo_init_angle_adjust == 1) Servo_init_angle_adjust_function(); //��������ʼλ��΢��ģʽ
			else Servo_angle_adjust_function();
			if(Check==0) //If self-check mode is not enabled //���û�������Լ�ģʽ
			{
				if (APP_ON_Flag)           Get_RC();         //Handle the APP remote commands //����APPң������
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //����PS2�ֱ���������
				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN������1������3(ROS)������5����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				//Click the user button to update the gyroscope zero
				//�����û������������������
				Key(); 
				Drive_Robot_Arm();//��е�ۿ���
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position
                //and the software failure flag is 0
				//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0
				if(Turn_Off(Voltage)==0) 
				 { 			
                     //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��

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
					 //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
					 switch(Car_Mode)
					 {
						case Mec_Car:       Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0 );  break;    //������	
						case Omni_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
						case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //������С��
						case Diff_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm,  0, 0, 0); break; //���ֲ���С��
					    case FourWheel_Car: Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0    );   break; //������
						case Tank_Car:      Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, 0, 0, 0);   break; 	
					 }

					 
				 }

				  
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //���Turn_Off(Voltage)����ֵΪ1�����������С�������˶���PWMֵ����Ϊ0
				 else	Set_Pwm(0,0,0,0,0); 
			 }	
		 }
			else Set_Pwm(0,0,0,0,0); //ˮ�û�̨�ƶ�ʱ�򣬲��������С�������˶���PWMֵ����Ϊ0
				
	 }			
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//�������ת����force_stop || forcestop
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
		//�������ת����	
		if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
		else 	                PWMB2=16799,PWMB1=16799-motor_b;
	
		//Forward and reverse control of motor
		//�������ת����	
		if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
		else 	                PWMC2=16799,PWMC1=16799-motor_c;

		//Forward and reverse control of motor
		//�������ת����
		if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
		else 	                PWMD2=16799,PWMD1=16799-motor_d;		
#endif

	//�������
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
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
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
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
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
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
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
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
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
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
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
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
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>15960)Pwm=15960;
	 if(Pwm<-15960)Pwm=-15960;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 if(Target == 0 && Bias == 0)Pwm = 0; 
	 if(stopkey_pressed){
        if(Pwm > 3000) Pwm = 3000;
		else if(Pwm < -3000) Pwm = -3000;

	 }
	 return Pwm; 
}
#if 1
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID2 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID3 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID4 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID5 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID6 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
#endif




/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	uint8_t Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
	 switch(Flag_Direction)  //Handle direction control commands //�������������
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
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת
		 else 		             Move_Z=0;                       //stop           //ֹͣ
	 }
	}	
	else //Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
	 switch(Flag_Direction) //Handle direction control commands //�������������
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
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
	}
	
	//Z-axis data conversion //Z������ת��
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//����
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //��PS2�ֱ�����������д���
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
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
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/


void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
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
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break;    //������	
			case Omni_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break; //ȫ����С��
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break;//������С��
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B;                                                                       break; //���ֲ���С��
		    case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D;   break; //������
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
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
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
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


#if 1
/**************************************************************************
�������ܣ���е�۹ؽڽǶ�ת��Ӧpwmֵ����
��ڲ�������е��Ŀ��ؽڽǶ�
����  ֵ���ؽڽǶȶ�Ӧ��pwmֵ
**************************************************************************/
int SERVO_PWM_VALUE(int id,float angle)
{
    #if 0
    float Ratio;
	int K=1000;
    if(id <= 2) Ratio=0.64;
	else  Ratio=0.424;
	int pwm_value;
	pwm_value=500+(angle*K*Ratio); //���Ŀ��
    return pwm_value;
	#else
    int K=1000;
	float Ratio=0.64;
	int pwm_value;
	pwm_value=(SERVO_INIT-angle*K*Ratio); //���Ŀ��
    return pwm_value;
	#endif
}
/**************************************************************************
�������ܣ���е�۹ؽڽǶ���λ����
��ڲ�������
����  ֵ����
**************************************************************************/
void moveit_angle_limit(void)
{ 
    Moveit_Angle1=target_limit_float(Moveit_Angle1,-1.57,1.57);
	Moveit_Angle2=target_limit_float(Moveit_Angle2,-1.57,1.57);
	Moveit_Angle3=target_limit_float(Moveit_Angle3,-1.57,1.57);
	Moveit_Angle4=target_limit_float(Moveit_Angle4,-0.3,1.57);
	Moveit_Angle5=target_limit_float(Moveit_Angle5,-1.57,1.57);
	Moveit_Angle6=target_limit_float(Moveit_Angle6,-0.7,0.7);  //��צ���˶���Χ��С
	
}
/**************************************************************************
�������ܣ���е�۹ؽ�PWMֵ��λ����
��ڲ�������
����  ֵ����
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
�������ܣ���е�۹ؽ��������ִ��룬ʹ��PIDλ�ÿ�������
��ڲ�������
����  ֵ����
**************************************************************************/


int pwm1,pwm2,pwm3,pwm4;
int pwm1_=1500,pwm2_=1500,pwm3_=1500,pwm4_=1500;

int cnt = 0;
uint32_t now;
uint8_t ServoSendCmd = 1;
int Servo1Status = 1,Servo2Status = 1,Servo3Status = 1;
void Drive_Robot_Arm(void)
{     
      moveit_angle_limit();		 //�ؽڽǶ����޷�
      Moveit_PWM1=  SERVO_PWM_VALUE(1,Moveit_Angle1)+Moveit_Angle1_init; //����Ŀ�껡�ȣ�������Ŀ��PWMֵ
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
	  

      Position1+=Velocity1;		   //�ٶȵĻ��֣��õ������λ��
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

