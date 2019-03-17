#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


typedef struct {
	      u8 MpuExist;      // MPU存在
	      u8 MagExist;      // MAG存在
	      u8 NrfExist;      // NRF存在
	      u8 MagIssue;      // MAG有问题
//        u8 ARMED;         // 电机解锁
	      u8 LockYaw;       // 航向锁定       
		  u8 calibratingA;  // 加速度采集
	      u8 calibratingM;  // 磁力计采集
	      u8 calibratingM_pre; //磁力计预采集
	      
	      u8 ParamSave;     // 参数保存标志
	      u8 FlightMode;	//飞行模式
	
	      u8 Loop_250Hz;	//4ms
	      u8 Loop_100Hz;	//10ms
	      u8 Loop_10Hz;		//50ms
	      u8 Loop_3s;		//3s
         }Flag_t;	//标志1
typedef struct {
	      u8 Loop1_250Hz;	//4ms
	      u8 Loop1_100Hz;	//10ms
	      u8 Loop1_10Hz;		//50ms
	      u8 Loop1_3s;		//3s
		  u8 Loop1_3_5s;		//3.5s
		  u8 Loop1_7s;	//7s
         }Flag1_t;	//标志2
typedef struct {
	      u8 Loop2_250Hz;	//4ms
	      u8 Loop2_100Hz;	//10ms
	      u8 Loop2_10Hz;		//50ms
	      u8 Loop2_3s;		//3s
		  u8 Loop2_3_5s;		//3.5s
		  u8 Loop2_7s;	//7s
         }Flag2_t;
typedef struct {
		  u8 Loop3_250Hz;	//4ms
		  u8 Loop3_100Hz;	//10ms
		  u8 Loop3_10Hz;		//50ms
		  u8 Loop3_1s;	//1.1s
	      u8 Loop3_2s;	//1.2s
	 	  u8 Loop3_3s;	//1.3s
		  u8 Loop3_4s;	//1.4s
	 	  u8 Loop3_5s;	//1.5s

		}Flag3_t;
//按键控制
typedef struct {
	      u8 Mark_0;	//执行第0个程序（A——>B区）
		  u8 Mark_1;	//执行第1个程序 (A<——>B区)
	      u8 Mark_2;	//执行第2个程序
		  u8 Mark_3;	//执行第3个程序

         }Mark_t;

extern Flag_t flag;	   		//中断判断标志位
extern Flag1_t flag1;	    //中断判断标志位
extern Flag2_t flag2;	   //中断判断标志位
extern Flag3_t flag3;	   //中断判断标志位

extern Mark_t mark;	  //程序执行标志位

void Bootloader_Set(void);
void InitBoard(void);
void Sensor_Init(void);
void Screen_Update(void);
void Motor_Conter_1(void);//自定义电机降速
void loop(void);
void Time_slice(void);	  //时间片0

void loop_1(void);
void Time_slice_1(void);  //时间片1

void loop_2(void);
void Time_slice_2(void);  //时间片2

void loop_3(void);
void Time_slice_3(void);  //时间片3

#endif /* __MultiRotor_app_H */



