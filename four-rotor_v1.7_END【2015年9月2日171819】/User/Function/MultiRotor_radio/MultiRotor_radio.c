/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��IMU.c
 * ����    ����̬����         
 * ʵ��ƽ̨��HT_Hawk
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team
**********************************************************************************/
#include "board_config.h"
#include "MultiRotor_radio.h"

int i,F,sw; 

unsigned char HtoEs_OutPut_Buffer[63] = {0};	   //���ڷ��ͻ�����
unsigned int CHK_SUM;  //У���


///////////////////////////////////////////////////////////////////
// 15ͨ������

float CH1_data  = 0;
float CH2_data  = 0;
float CH3_data  = 0;
float CH4_data  = 0;
float CH5_data  = 0;
float CH6_data  = 0;
float CH7_data  = 0;  //15ͨ����������
float CH8_data  = 0;
float CH9_data  = 0;
float CH10_data = 0;
float CH11_data = 0;
float CH12_data = 0;
float CH13_data = 0;
float CH14_data = 0;
float CH15_data = 0;

///////////////////////////////////////////////////////////////////
//  GPS ����

float Longitude_val;  //������ֵ
float Latitude_Val ;  //γ����ֵ

float Altitude_Val;  //�߶���ֵ
float Dir_Val;       //��λ��ֵ
float SPD_Val;       //�ٶ���ֵ

unsigned char Satellite_Val;   //���Ǹ���
unsigned int  Voltage_Val;     //��ص�ѹ
unsigned int  Temperture_Val;  //�¶���ֵ

unsigned char Longitude_WE; //��־���ȷ���true=W��false=E��
unsigned char Latitude_NS;  //��־γ�ȷ���true=N��false=S��
unsigned char Location_Sta; //��λ״̬��ʶ

///////////////////////////////////////////////////////////////////
// PID ����

int Pitch_PID_P;
int Pitch_PID_I;
int Pitch_PID_D;

int Roll_PID_P;
int Roll_PID_I;
int Roll_PID_D;

int Yaw_PID_P;
int Yaw_PID_I;
int Yaw_PID_D;

int Alt_PID_P;
int Alt_PID_I;
int Alt_PID_D;

int Pos_PID_P;
int Pos_PID_I;
int Pos_PID_D;

//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//���ɶ���ͨ������֡
unsigned char HtoEs_Chart_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x3F; //֡���� 63�ֽ�
		HtoEs_OutPut_Buffer[1] = 0x01; //������
		
	  CH1_data = (float)sensor.acc.origin.x;
	  CH2_data = (float)sensor.acc.origin.y;
	  CH3_data = (float)sensor.acc.origin.z;
	  CH4_data = (float)sensor.gyro.origin.x;
	  CH5_data = (float)sensor.gyro.origin.y;
	  CH6_data = (float)sensor.gyro.origin.z;
	  CH7_data = (float)MAG[0];
	  CH8_data = (float)MAG[1];
	  CH9_data = (float)MAG[2];
	  CH10_data = (float)US100_Alt;
	  CH11_data = (float)0;
	  CH12_data = (float)0;
	
		Float2Byte(&CH1_data ,HtoEs_OutPut_Buffer,2);
		Float2Byte(&CH2_data ,HtoEs_OutPut_Buffer,6);
		Float2Byte(&CH3_data ,HtoEs_OutPut_Buffer,10);
		Float2Byte(&CH4_data ,HtoEs_OutPut_Buffer,14);
		Float2Byte(&CH5_data ,HtoEs_OutPut_Buffer,18);
		Float2Byte(&CH6_data ,HtoEs_OutPut_Buffer,22);
		Float2Byte(&CH7_data ,HtoEs_OutPut_Buffer,26);
		Float2Byte(&CH8_data ,HtoEs_OutPut_Buffer,30);
		Float2Byte(&CH9_data ,HtoEs_OutPut_Buffer,34);
		Float2Byte(&CH10_data,HtoEs_OutPut_Buffer,38);
		Float2Byte(&CH11_data,HtoEs_OutPut_Buffer,42);
		Float2Byte(&CH12_data,HtoEs_OutPut_Buffer,46);
		Float2Byte(&CH13_data,HtoEs_OutPut_Buffer,50);
		Float2Byte(&CH14_data,HtoEs_OutPut_Buffer,54);
		Float2Byte(&CH15_data,HtoEs_OutPut_Buffer,58);
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 62; i++)  //�����
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[62] = CHK_SUM % 255; //����У��ֵ
 
	  return 63; 
}

//����GPS����֡
unsigned char HtoEs_GPS_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x1D; //֡���� 29�ֽ�
		HtoEs_OutPut_Buffer[1] = 0x02; //������
	
	  Float2Byte(&Longitude_val ,HtoEs_OutPut_Buffer,2);  //����
	  Float2Byte(&Latitude_Val  ,HtoEs_OutPut_Buffer,6);  //γ��
	  Float2Byte(&Altitude_Val  ,HtoEs_OutPut_Buffer,10); //�߶�
	  Float2Byte(&Dir_Val ,HtoEs_OutPut_Buffer,14);       //��λ��
	  Float2Byte(&SPD_Val ,HtoEs_OutPut_Buffer,18);       //�ٶ�
	
	  HtoEs_OutPut_Buffer[22] = (Voltage_Val & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[23] = (Voltage_Val & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[24] = (Temperture_Val & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[25] = (Temperture_Val & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[26] = Satellite_Val; //���Ǹ���
	
	//============================================================================
	  
		HtoEs_OutPut_Buffer[27] = 0; //�Ƚ�״̬��ʶ���
		
		if( Location_Sta ) //��λģʽ
			HtoEs_OutPut_Buffer[27] |= 0x01; //�ø�
		else               //����ģʽ
			HtoEs_OutPut_Buffer[27] &= 0xFE; //����
		
		
		if( Longitude_WE == 'W' )  //���ȷ���
			HtoEs_OutPut_Buffer[27] |= 0x02;  //W,����
		else if( Longitude_WE == 'E' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFD;  //E,����
 
		if( Latitude_NS == 'N' )  //γ�ȷ���
			HtoEs_OutPut_Buffer[27] |= 0x04;  //N,��γ
		else if( Latitude_NS == 'S' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFB;  //S,��γ
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 28; i++)  //�����
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[28] = CHK_SUM % 255; //����У��ֵ
	
	  return 29; 
}

//������̬����֡
unsigned char HtoEs_Attitude_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x0F; //֡���� 15�ֽ�
		HtoEs_OutPut_Buffer[1] = 0x03; //������
	
	  Float2Byte(&AngE.Pitch,HtoEs_OutPut_Buffer,2);  //����
	  Float2Byte(&AngE.Roll ,HtoEs_OutPut_Buffer,6);  //���
	  Float2Byte(&AngE.Yaw  ,HtoEs_OutPut_Buffer,10); //����
	  
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 14; i++)  //�����
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[14] = CHK_SUM % 255; //����У��ֵ
		
	  return 15; 
}

//����RCͨ������֡
unsigned char HtoEs_RC_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x15; //֡���� 15�ֽ�
		HtoEs_OutPut_Buffer[1] = 0x04; //������
	 
	  HtoEs_OutPut_Buffer[2] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[3] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[4] = (RC_Data.rc_data[1] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[5] = (RC_Data.rc_data[1] & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[6] = (RC_Data.rc_data[2] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[7] = (RC_Data.rc_data[2] & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[8] = (RC_Data.rc_data[3] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[9] = (RC_Data.rc_data[3] & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[10] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[11] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[12] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[13] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[14] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[15] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[16] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[17] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[18] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[19] = (RC_Data.rc_data[0] & 0x00FF) ;      //ȡ��8λ
	 
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 20; i++)  //�����
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[20] = CHK_SUM % 255; //����У��ֵ
	
	  return 21; 
}

//����PID����֡
unsigned char HtoEs_PID_Data_Generate(void)
{
	  unsigned char i;
	
	  Pitch_PID_P = ctrl.pitch.core.kp * 100;
	  Pitch_PID_I = ctrl.pitch.core.ki * 100;
	  Pitch_PID_D = ctrl.pitch.core.kd * 100;
	  Roll_PID_P  = ctrl.roll.core.kp * 100;
	  Roll_PID_I  = ctrl.roll.core.ki * 100;
	  Roll_PID_D  = ctrl.roll.core.kd * 100;
	  Yaw_PID_P   = ctrl.yaw.core.kp * 100;
	  Yaw_PID_I   = ctrl.yaw.core.ki * 100;
	  Yaw_PID_D   = ctrl.yaw.core.kd * 100;
	
	  HtoEs_OutPut_Buffer[0] = 0x21; //֡���� 15�ֽ�
		HtoEs_OutPut_Buffer[1] = 0x05; //������
	
	  HtoEs_OutPut_Buffer[2] = (Pitch_PID_P & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[3] = (Pitch_PID_P & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[4] = (Pitch_PID_I & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[5] = (Pitch_PID_I & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[6] = (Pitch_PID_D & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[7] = (Pitch_PID_D & 0x00FF) ;      //ȡ��8λ
	
	  HtoEs_OutPut_Buffer[8] = (Roll_PID_P & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[9] = (Roll_PID_P & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[10] = (Roll_PID_I & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[11] = (Roll_PID_I & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[12] = (Roll_PID_D & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[13] = (Roll_PID_D & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[14] = (Yaw_PID_P & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[15] = (Yaw_PID_P & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[16] = (Yaw_PID_I & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[17] = (Yaw_PID_I & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[18] = (Yaw_PID_D & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[19] = (Yaw_PID_D & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[20] = (Alt_PID_P & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[21] = (Alt_PID_P & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[22] = (Alt_PID_I & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[23] = (Alt_PID_I & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[24] = (Alt_PID_D & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[25] = (Alt_PID_D & 0x00FF) ;      //ȡ��8λ
		
		HtoEs_OutPut_Buffer[26] = (Pos_PID_P & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[27] = (Pos_PID_P & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[28] = (Pos_PID_I & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[29] = (Pos_PID_I & 0x00FF) ;      //ȡ��8λ
	  HtoEs_OutPut_Buffer[30] = (Pos_PID_D & 0xFF00) >> 8 ; //ȡ��8λ
	  HtoEs_OutPut_Buffer[31] = (Pos_PID_D & 0x00FF) ;      //ȡ��8λ
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 32; i++)  //�����
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[32] = CHK_SUM % 255; //����У��ֵ
	
	  return 33; 
}

// //���ɴ���������֡
// unsigned char HtoEs_Senosrs_Data_Generate(void)
// {
// 	  unsigned char i;
// 	
// 	  HtoEs_OutPut_Buffer[0] = 0x27; //֡���� 15�ֽ�
// 		HtoEs_OutPut_Buffer[1] = 0x06; //������
// 	
// 	  Float2Byte(&Gyro_X  ,HtoEs_OutPut_Buffer,2);
// 		Float2Byte(&Gyro_Y  ,HtoEs_OutPut_Buffer,6);
// 		Float2Byte(&Gyro_Z  ,HtoEs_OutPut_Buffer,10);
// 		Float2Byte(&Accel_X ,HtoEs_OutPut_Buffer,14);
// 		Float2Byte(&Accel_Y ,HtoEs_OutPut_Buffer,18);
// 		Float2Byte(&Accel_Z ,HtoEs_OutPut_Buffer,22);
// 		Float2Byte(&Mag_X   ,HtoEs_OutPut_Buffer,26);
// 		Float2Byte(&Mag_Y   ,HtoEs_OutPut_Buffer,30);
// 		Float2Byte(&Mag_Z   ,HtoEs_OutPut_Buffer,34);
// 	
// //============================================================================	
// 		
// 		CHK_SUM =0; 
// 	
// 	  for(i = 0 ; i < 38; i++)  //�����
// 			CHK_SUM += HtoEs_OutPut_Buffer[i];
// 		
// 		HtoEs_OutPut_Buffer[38] = CHK_SUM % 255; //����У��ֵ	
// 	
// 	  return 39; 
// }

s8 fg=10;

void mavlink(void)
{
	switch(sw) //ѭ�����͸�ģ������
	{
			case 1: F = HtoEs_Chart_Data_Generate();    break;  //���Զ���ͨ��,�����跢���ֽ���
			case 2: F = HtoEs_RC_Data_Generate();       break;  //����RCͨ���������跢���ֽ���		 
      case 3: F = HtoEs_Attitude_Data_Generate(); break;  //������̬,�����跢���ֽ���	
      case 4: if(fg ==10 ) F = HtoEs_PID_Data_Generate(); 
				      if(fg<10 && fg >0){
								fg--; 
								F = HtoEs_PID_Data_Generate();       //����PID�����������跢���ֽ���		
			        }				
							break;  
      case 5: F = HtoEs_GPS_Data_Generate(); 		  break;     //����GPS,�����跢���ֽ���
    //case 6: F = HtoEs_Senosrs_Data_Generate();  break;  //���Դ������궨�������跢���ֽ���					
			case 6: sw = 0; break; 
	}
	sw++; 
		
	usb_SendDataToHost(&HtoEs_OutPut_Buffer[0], F);
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : UsbCmdPro
**���� : USB���ݽ���
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void UsbCmdPro(void)
{
  u8 ucData,ucNum;
  static u16 aCmdBuf[32];
	static u16 usPos;
	static u16 Free_heart=0;
	
	// ��������
	Free_heart++;  
	if(Free_heart>=60) {
		usPos = 0;
		Free_heart = 60;
 		array_assign(aCmdBuf,0,32);
  }
	
	// ��USB�ڶ�ȡһ���ֽ� ucNum��Ŷ������ֽڸ��� 
	ucData = usb_GetRxByte(&ucNum);	
	
	// û�н��յ���� �˳�
	if (ucNum == 0)		return;
	
	Free_heart=0;
	
 	// ���յ������ݷ��뻺��
 	aCmdBuf[usPos++] = ucData;
  
	if(usPos>=31) usPos=31;
	
	// ���ݽ���
	if(fg!=10) Data_Parser(aCmdBuf);
	fg=5;
}


