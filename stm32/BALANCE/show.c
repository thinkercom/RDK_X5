#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
extern Encoder OriginalEncoder;     //������ԭʼ����
extern int S1Pos;
extern int protect_count;
extern int stopkey_count;

/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count=25;
bool show_imu = 0;
bool show_servo = 0;
bool motor_test = 0;

int Page_ID;
void show_task(void *pvParameters)
{
   uint32_t lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		static int Servo_adjust_timecount;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������
		
		//����ʱ���������ݷ�������������
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //����������
		else if(Buzzer_count==5)Buzzer=0;

        if(Beep_Control_Flag)
		{
			Buzzer = 1;
			Servo_adjust_timecount++;
			if(Servo_adjust_timecount == 10)
			{
				Beep_Control_Flag = 0;
				Buzzer = 0;
				Servo_adjust_timecount = 0;
				OLED_Refresh_Gram(); //ˢ����Ļ
                OLED_Clear();  //�����Ļ
		        OLED_Refresh_Gram();
			}
		}
        if(Long_Press() == 1)
     	{
		    Page_ID++;
			if(Page_ID == 1){
                show_imu = 1;
				show_servo = 0;
				motor_test = 0;
			}
            else if(Page_ID == 2)
            {
                show_imu = 0;
				show_servo = 1;
				motor_test = 0;
			}
            else if(Page_ID == 3)
			{
                show_imu = 0;
				show_servo = 0;
				motor_test = 1;
			  
			}

            else if(Page_ID == 4)
			{
                show_imu = 0;
				show_servo = 0;
				motor_test = 0;
			    Page_ID = 0;
			}
			
            OLED_Refresh_Gram(); //ˢ����Ļ
            OLED_Clear();  //�����Ļ
	        OLED_Refresh_Gram();
	    }
		
		//Read the battery voltage //��ȡ��ص�ѹ
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��

        if(Voltage>=12.6)Voltage=12.6;//���ⳬ��ص�ѹ
        else if(Voltage<10.5)Voltage=Voltage-0.5;//�͵�����ʱ����ʾ����һ�㼰ʱ���
        if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V���͵���ʱ��������һ�α���
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V��С����ֹ����ʱ�������ڶ��α���
	    oled_show(); //Tasks are displayed on the screen //��ʾ����ʾ����
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int dis_L,dis_R, dis_M3,dis_M4;

void oled_show(void)
{  
	 int Car_Mode_Show;
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); 
	 if(Car_Mode_Show>5)Car_Mode_Show=5; 
	  //Car_Mode_Show=0;
	
	 Voltage_Show=Voltage*100; 
	if((Servo_init_angle_adjust == 0 && show_imu == 0 && show_servo == 0) ||  motor_test == 1)//��������
   {
	 if(Check==0)//The car displays normally when the self-check mode is not enabled //û�п����Լ�ģʽʱС��������ʾ
	 {	
		 //The first line of the display displays the content//
		 //��ʾ����1����ʾ����//

	
		 if(motor_test == 1){
		 
			 Move_X = 0.3;
			 RosCmdUpdate++;
		  }
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
			case Omni_Car:      OLED_ShowString(0,0,"Omni"); break; 
			case Akm_Car:       OLED_ShowString(0,0,"Akm "); break; 
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; 
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
            case Tank_Car:      OLED_ShowString(0,0,"Tank"); break;
			
		 }
		 
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		 {
			 //The Mec_car and omni_car show Z-axis angular velocity
			 //���֡�ȫ����С����ʾZ����ٶ�
			 OLED_ShowString(55,0,"GZ");
			 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
			 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		
		 }
         

		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car || Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
			 //�����������١��������Ĵ�����ʾ���������
			 OLED_ShowString(50,0,"BIAS ");
			 if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],4,12);  //Zero-drift data of gyroscope Z axis
			 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],4,12);	//������z�����Ư������	

		 }
		 //The second line of the display displays the content//
		 //��ʾ����2����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
			//���֡�ȫ���֡���������ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			OLED_ShowString(0,10,"M1");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			#if SHOW_SPEED == 1
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);

			 #elif SHOW_SPEED == 0
			
									OLED_ShowNumber(75,10,abs(OriginalEncoder.E),7,12);		

			 #else

			                        dis_R =   (int)(abs(OriginalEncoder.F)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,10,dis_R,5,12); 	
			 #endif
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 OLED_ShowString(00,10,"GYRO_Z:");
			 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),  OLED_ShowNumber(75,10,-gyro[2],5,12);    ///mark
											
			 else            OLED_ShowString(60,10,"+"),  OLED_ShowNumber(75,10, gyro[2],5,12);		//z������������	
		 }
		 //Lines 3 and 4 of the display screen display content//
		 //��ʾ����3��4����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
			//���֡�ȫ���֡���������ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			OLED_ShowString(0,20,"M2");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			#if SHOW_SPEED == 1
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);


             #elif SHOW_SPEED == 0
			
									OLED_ShowNumber(75,20,abs(OriginalEncoder.F),7,12);		

			 #else

			                        dis_R =   (int)(abs(OriginalEncoder.F)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,20,dis_R,5,12); 	
			 #endif


			
			
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
			//���֡�ȫ���֡���������ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			OLED_ShowString(0,30,"M3");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
			#if SHOW_SPEED== 1 
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			#elif SHOW_SPEED== 0 

                                    OLED_ShowNumber(75,30, abs(OriginalEncoder.G),7,12);
			#else

			                        dis_M3 =   (int)(abs(OriginalEncoder.G)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,30,dis_R,5,12); 	
			#endif
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
			 //�����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			 OLED_ShowString(0,20,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
			 #if SHOW_SPEED == 1
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 #elif SHOW_SPEED == 0
			
									OLED_ShowNumber(75,20,abs(OriginalEncoder.E),7,12);		

			 #else

			                        dis_R =   (int)(abs(OriginalEncoder.E)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,20,dis_R,5,12); 	
			 #endif
			 //Akm_Car, D Tank_Car Display the target speed and current actual speed of motor B
			 //�����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			 OLED_ShowString(0,30,"R:");
			 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
			 else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
			 #if SHOW_SPEED	== 1
			 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);
             #elif SHOW_SPEED == 0
			
									OLED_ShowNumber(75,30,abs(OriginalEncoder.F),7,12);		

			 #else

			                        dis_R =   (int)(abs(OriginalEncoder.F)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,30,dis_R,5,12); 	
			 #endif

		 }
		
		 
		 //Line 5 of the display displays the content//
		 //��ʾ����5����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
		 {
			  //Mec_Car Display the target speed and current actual speed of motor D
				//����С����ʾ���D��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,40,"M4");
				if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
															OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
				else                 	OLED_ShowString(15,40,"+"),
															OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 


				#if SHOW_SPEED == 1
				if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
				#elif SHOW_SPEED== 0 

                                    OLED_ShowNumber(75,40, abs(OriginalEncoder.H),7,12);
			    #else

			                        dis_M4 =   (int)(abs(OriginalEncoder.H)*1000*Wheel_perimeter/Encoder_precision); 
									OLED_ShowNumber(75,40,dis_R,5,12); 	
			    #endif
		 }
		 else if(Car_Mode==Omni_Car)
		 {
			  // The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
				//ȫ����С����ʾZ����ٶ�(�Ŵ�1000��)����λrad/s
				OLED_ShowString(0,40,"MOVE_Z"); 			
				if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																						OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
				else                              	OLED_ShowString(60,40,"+"),
																						OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
		 }
		 else if(Car_Mode==Akm_Car)
		 {
			  //Akm_Car displays the PWM value of the Servo
				//������С����ʾ�����PWM����ֵ
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(80,40,-Servo,4,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(80,40, Servo,4,12); 
		 }
		 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
			 //����С�����Ĵ�����ʾ���ҵ����PWM����ֵ
			 OLED_ShowString(00,40,"M1");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,5,12);
			 else                 	 OLED_ShowString(20,40,"+"),
															 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,5,12); 
			 OLED_ShowString(60,40,"M2");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,5,12);
			 else                 	 OLED_ShowString(80,40,"+"),
															 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,5,12);
		 }
		
		 //Displays the current control mode //��ʾ��ǰ����ģʽ
		 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
		 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
		 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
		 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
		 else if ((Usart1_ON_Flag || Usart5_ON_Flag)==1) OLED_ShowString(0,50,"USART");
		 else                       OLED_ShowString(0,50,"ROS  ");
			
		 //Displays whether controls are allowed in the current car
		 //��ʾ��ǰС���Ƿ��������
		 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
		 else                      OLED_ShowString(45,50,"OFF"); 
			
								    OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
			                        OLED_ShowString(88,50,".");
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			                        OLED_ShowString(110,50,"V");
		 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		}	
     }
   else if(show_imu == 1)
   	{
		//=============��һ��=======================//	
		OLED_ShowString(0,0,"gyro_x:");
		if( gyro[0 ]<0)	
		{
			OLED_ShowString(70,0,"-");
			OLED_ShowNumber(80,0, -gyro[0],5,12);
		}
		else
		{
			OLED_ShowString(70,0,"+");
			OLED_ShowNumber(80,0, gyro[0],5,12);
		}
		//=============�ڶ���=======================//	
		OLED_ShowString(0,10,"gyro_y:");
		if( gyro[1]<0)	
		{
			OLED_ShowString(70,10,"-");
			OLED_ShowNumber(80,10, -gyro[1],5,12);
		}
		else
		{
			OLED_ShowString(70,10,"+");
			OLED_ShowNumber(80,10, gyro[1],5,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,20,"gyro_z:");
		if( gyro[2]<0)	
		{
			OLED_ShowString(70,20,"-");
			OLED_ShowNumber(80,20, -gyro[2],5,12);
		}
		else
		{
			OLED_ShowString(70,20,"+");
			OLED_ShowNumber(80,20, gyro[2],5,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,30,"accel_x:");
		if( accel[0]<0)	
		{
			OLED_ShowString(70,30,"-");
			OLED_ShowNumber(80,30, -accel[0]*100/1671,5,12);
		}
		else
		{
			OLED_ShowString(70,30,"+");
			OLED_ShowNumber(80,30, accel[0]*100/1671,5,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,40,"accel_y:");
		if( accel[1]<0)	
		{
			OLED_ShowString(70,40,"-");
			OLED_ShowNumber(80,40, -accel[1]*100/1671,5,12);
		}
		else
		{
			OLED_ShowString(70,40,"+");
			OLED_ShowNumber(80,40, accel[1]*100/1671,5,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,50,"accel_z:");
		if( accel[2]<0)	
		{
			OLED_ShowString(70,50,"-");
			OLED_ShowNumber(80,50, -accel[2]*100/1671,5,12);
		}
		else
		{
			OLED_ShowString(70,50,"+");
			OLED_ShowNumber(80,50, accel[2]*100/1671,5,12);
		}
	 }

   else if (show_servo == 1)
   	{
		//=============��һ��=======================//	
		OLED_ShowString(0,0,"Position1:");
		
		OLED_ShowNumber(100,0, Position1,4,12);
		
		//=============�ڶ���=======================//	
		OLED_ShowString(0,10,"Position2:");
		
		OLED_ShowNumber(100,10, Position2,4,12);
		//=============������=======================//	
		OLED_ShowString(0,20,"Position3:");
		
		OLED_ShowNumber(100,20, Position3,4,12);
		//=============������=======================//	
		OLED_ShowString(0,30,"Position4:");
		
		OLED_ShowNumber(100,30, Position4,4,12);
		//=============������=======================//	
		OLED_ShowString(0,40,"Position5:");
		
		OLED_ShowNumber(100,40, Position5,4,12);
		//=============������=======================//	
		OLED_ShowString(0,50,"Position6:");
		
		OLED_ShowNumber(100,50, Position6,4,12);
	 }
   else 
	{
		//=============��һ��=======================//	
		OLED_ShowString(0,0,"Angle1_init:");
		if( Moveit_Angle1_init<0)	
		{
			OLED_ShowString(100,0,"-");
			OLED_ShowNumber(110,0, -Moveit_Angle1_init,5,12);
		}
		else
		{
			OLED_ShowString(100,00,"+");
			OLED_ShowNumber(110,0, Moveit_Angle1_init,5,12);
		}
		//=============�ڶ���=======================//	
		OLED_ShowString(0,10,"Angle2_init:");
		if( Moveit_Angle2_init<0)	
		{
			OLED_ShowString(100,10,"-");
			OLED_ShowNumber(110,10, -Moveit_Angle2_init,3,12);
		}
		else
		{
			OLED_ShowString(100,10,"+");
			OLED_ShowNumber(110,10, Moveit_Angle2_init,3,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,20,"Angle3_init:");
		if( Moveit_Angle3_init<0)	
		{
			OLED_ShowString(100,20,"-");
			OLED_ShowNumber(110,20, -Moveit_Angle3_init,3,12);
		}
		else
		{
			OLED_ShowString(100,20,"+");
			OLED_ShowNumber(110,20, Moveit_Angle3_init,3,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,30,"Angle4_init:");
		if( Moveit_Angle4_init<0)	
		{
			OLED_ShowString(100,30,"-");
			OLED_ShowNumber(110,30, -Moveit_Angle4_init,3,12);
		}
		else
		{
			OLED_ShowString(100,30,"+");
			OLED_ShowNumber(110,30, Moveit_Angle4_init,3,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,40,"Angle5_init:");
		if( Moveit_Angle5_init<0)	
		{
			OLED_ShowString(100,40,"-");
			OLED_ShowNumber(110,40, -Moveit_Angle5_init,3,12);
		}
		else
		{
			OLED_ShowString(100,40,"+");
			OLED_ShowNumber(110,40, Moveit_Angle5_init,3,12);
		}
		//=============������=======================//	
		OLED_ShowString(0,50,"Angle6_init:");
		if( Moveit_Angle6_init<0)	
		{
			OLED_ShowString(100,50,"-");
			OLED_ShowNumber(110,50, -Moveit_Angle6_init,3,12);
		}
		else
		{
			OLED_ShowString(100,50,"+");
			OLED_ShowNumber(110,50, Moveit_Angle6_init,3,12);
		}
	 }

		OLED_Refresh_Gram();
}
