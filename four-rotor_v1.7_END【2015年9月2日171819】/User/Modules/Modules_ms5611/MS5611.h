#ifndef __MS5611_H
#define	__MS5611_H
#include "stm32f10x.h"
/* MPU6050 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611µÄµØÖ·
#define MS561101BA_RST             0x1E  //cmd ¸´Î»

#define	MS561101BA_D2_OSR_4096   0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096   0x48

#define MS5611_OSR256					 		 0x40
#define MS5611_OSR512					 		 0x42
#define MS5611_OSR1024					   0x44
#define MS5611_OSR2048					   0x46
#define MS5611_OSR4096					   0x48
#define FILTER_num 20



#define MS5611_ADDR             0xEE   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096


u8  MS5611_init(void);
void MS561101BA_getTemperature(void);
float Get_High(void);
void Sethigh_Mode(void);
#endif
