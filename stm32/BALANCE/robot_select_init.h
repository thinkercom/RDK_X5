#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //轮距 麦轮车为半轮距
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //轴距 麦轮车为半轴距	
  float GearRatio;           //Motor_gear_ratio //电机减速比
  int EncoderAccuracy;     //Number_of_encoder_lines //编码器精度(编码器线数)
  float WheelDiameter;     //Diameter of driving wheel //主动轮直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向轮小车旋转半径	
}Robot_Parament_InitTypeDef;

// Encoder structure
//编码器结构体
#define SHOW_SPEED 1
typedef struct  
{
 
  int A;      
  int B; 
  int C; 
  int D; 
  #if SHOW_SPEED != 1
  long E;      
  long F; 
  long G; 
  long H; 

  #endif
}Encoder;

//The minimum turning radius of Ackermann models is determined by the mechanical structure: 
//the maximum Angle of the wheelbase, wheelbase and front wheels
//阿克曼车型的最小转弯半径，由机械结构决定：轮距、轴距、前轮最大转角
#define MINI_AKM_MIN_TURN_RADIUS 0.350f 
#define T10     0
#define TR300   0
#define T200    0
#define WT600   0
#define TS100   0
#define TS800   0
#define TS100P  1

#define T500        0
#define C500_4WD    0
#define C400_4WD    0
#define C100_4WD    0
#define C100PRO_4WD 1
#define C1000_4WD   0
#define TS900       0
#define T900        0
#define TS500       0

#define C300_MEC    1
#define C400_MEC    0
#define C500_MEC    0
#define C600_MEC    0
#define C800_MEC    0



#define HIGH_POWER_DRIVER_V2     0
#define IMU_MIRROR               0

#if T500
#define Four_Mortor_wheelSpacing 0.322f 
#elif TS500
#define Four_Mortor_wheelSpacing 0.314f 
#elif C400_4WD
#define Four_Mortor_wheelSpacing 0.258f 
#elif C500_4WD
#define Four_Mortor_wheelSpacing 0.28f 
#elif  C100_4WD
#define Four_Mortor_wheelSpacing 0.198f 
#elif  C100PRO_4WD
#define Four_Mortor_wheelSpacing 0.240f 
#elif  C1000_4WD
#define Four_Mortor_wheelSpacing 0.64f 
#elif  TS900
#define Four_Mortor_wheelSpacing 0.312f 
#elif   T900
#define Four_Mortor_wheelSpacing 0.312f 
#else
#define Four_Mortor_wheelSpacing 0.198f 
#endif

#if TR300
#define Tank_wheelSpacing        0.197f  
#elif T10
#define Tank_wheelSpacing        0.148f 
#elif T200
#define Tank_wheelSpacing        0.245f 
#elif WT600
#define Tank_wheelSpacing        0.245f 
#elif TS100P
#define Tank_wheelSpacing        0.205f 
#elif TS800
#define Tank_wheelSpacing        0.312f 
#else
#define Tank_wheelSpacing        0.38f  
#endif

#if C500_MEC
#define MEC_axlespacing          0.215f
#define MEC_wheelspacing         0.175f
#elif C400_MEC
#define MEC_axlespacing          0.117f
#define MEC_wheelspacing         0.128f
#elif C300_MEC
#define MEC_axlespacing          0.092f
#define MEC_wheelspacing         0.108f
#elif C800_MEC
#define MEC_axlespacing          0.185f
#define MEC_wheelspacing         0.160f

#else
#define MEC_axlespacing          0.117f
#define MEC_wheelspacing         0.128f

#endif

#define Akm_axlespacing           0.155f
#define Diff_axlespacing          0.155f
#if T500
  #define Four_Mortor__axlespacing  0.397f
#elif TS500
  #define Four_Mortor__axlespacing  0.525f
#elif C400_4WD
  #define Four_Mortor__axlespacing  0.232f  //0.30f

#elif C500_4WD
  #define Four_Mortor__axlespacing  0.24f

#elif C100_4WD
  #define Four_Mortor__axlespacing  0.098f
#elif C100PRO_4WD
  #define Four_Mortor__axlespacing  0.168f
#elif C1000_4WD
  #define Four_Mortor__axlespacing  0.56f
#elif  TS900
#define Four_Mortor__axlespacing    0.345f  
#elif T900
  #define Four_Mortor__axlespacing  0.515f
#else 
  #define Four_Mortor__axlespacing  0.564f
#endif

#if TR300
#define Tank_axlespacing          0.242f 
#endif


#define Akm_wheelspacing         0.155f
#define Diff_wheelSpacing        0.155f

//Motor_gear_ratio
//电机减速比
#define   GEAR_5F     5 
#define   GEAR_10F    10.41f 
#define   GEAR_18F    18 
#define   GEAR_20F    20 
#define   GEAR_30F    30 
#define   GEAR_28F    28 
#define   GEAR_30     30

#define   GEAR_31_4F  31.4f 
#define   GEAR_36F    36.072f

#define   GEAR_50F    50 
#define   GEAR_56F    56 

#define   GEAR_75F    75  
#define   GEAR_90F    90  


#define   M1_REVERSE    1
#define   M2_REVERSE    1
#define   M3_REVERSE    0
#define   M4_REVERSE    0

#define   HALL1_REVERSE  0
#define   HALL2_REVERSE  0
#define   HALL3_REVERSE  0
#define   HALL4_REVERSE  0



//Number_of_encoder_lines
//编码器精度
#define		Photoelectric_1000 1000
#define		Photoelectric_500  500
#define		Photoelectric_448  448
#define		Photoelectric_400  400
#define	    Hall_16            16
#define	    Hall_13            13
#define	    Hall_11            11

//Mecanum wheel tire diameter series
//麦轮轮胎直径
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_97  0.097f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 
//Omni wheel tire diameter series
//轮径全向轮直径系列
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//Black tire, tank_car wheel diameter
//黑色轮胎、履带车轮直径
#if T500
  #define	  Black_WheelDiameter   0.0583f   
#elif TS500
  #define	  Black_WheelDiameter   0.0426f
#elif C400_4WD
  #define	  Black_WheelDiameter   0.130f
#elif C500_4WD
  #define	  Black_WheelDiameter   0.151f 
#elif C100_4WD || C100PRO_4WD
  #define	  Black_WheelDiameter   0.0644f   
#elif C1000_4WD
  #define	  Black_WheelDiameter   0.34f
#elif TS900
  #define	  Black_WheelDiameter   0.0426f

#elif T900
  #define	  Black_WheelDiameter   0.05243f
#else 
  #define	  Black_WheelDiameter   0.05243f//0.0494f  
#endif

#if TR300
  #define	  Tank_WheelDiameter    0.0566f
#elif T10 
  #define	  Tank_WheelDiameter    0.0583f
#elif T200 
  #define	  Tank_WheelDiameter    0.0566f
#elif WT600 
  #define	  Tank_WheelDiameter    0.0420f
#elif TS100P 
  #define	  Tank_WheelDiameter    0.04354f
#elif TS800 
  #define	  Tank_WheelDiameter    0.0583f
#else
  #define	  Tank_WheelDiameter    0.0927f
#endif

//Rotation radius of omnidirectional trolley
//全向轮小车旋转半径
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

//The encoder octave depends on the encoder initialization Settings
//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples  4
//Encoder data reading frequency
//编码器数据读取频率
#define   CONTROL_FREQUENCY 100
//#define PI 3.1415f  //PI //圆周率

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
