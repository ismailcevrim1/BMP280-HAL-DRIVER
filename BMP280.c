/*
 * BMP280.c
 *
 *  Created on: 8 Kas 2021
 *      Author: ismail Cevrim
 */
#include "BMP280.h"

/* Description Of The Compensation Parameters  */
unsigned short dig_T1;
signed short dig_T2;
signed short dig_T3;
unsigned short dig_P1;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;

/* Description of the Variables*/
uint8_t Stage;
uint8_t DataAddr = 0xF7;
uint8_t RecvBuff[6] = {0};
uint32_t UnComp_Pressure_Array[3];
uint32_t UnComp_Pressure = 0;
uint32_t UnComp_Temperature_Array[3];
uint32_t UnComp_Temperature = 0;

float Yn = 0;
float Yn1 = 0;
float Xn = 0;
float Xn1 = 0;

float temperature, pressure, altitude;

/* Sets the parameters of the Sensor's Resolution*/
void BMP280_Init(void){

	uint8_t DataBuff[2];
	DataBuff[0] = 0x57;
	DataBuff[1] = 0x1C;

	HAL_I2C_Mem_Write(&hi2c1 , BMP280_WRITE_REGISTER_ADDRESS , 0xF4 , 1 , &DataBuff[0] , 1 , 1000);
	HAL_I2C_Mem_Write(&hi2c1 , BMP280_WRITE_REGISTER_ADDRESS , 0xF5 , 1 , &DataBuff[1] , 1 , 1000);
	HAL_Delay(10);

	BMP280_Read_Calibration_Values();

}

void BMP280_Read_Calibration_Values(void){

	uint8_t CompAddress = 0x88;
	uint8_t CalibBuff[24];

	HAL_I2C_Master_Transmit(&hi2c1 , BMP280_WRITE_REGISTER_ADDRESS , &CompAddress , 1 , 1000);
	HAL_I2C_Master_Receive(&hi2c1 , BMP280_READ_REGISTER_ADDRESS , &CalibBuff[0] , 24 , 1000);

	dig_T1 = (CalibBuff[0])  + (CalibBuff[1]  << 8);
	dig_T2 = (CalibBuff[2])  + (CalibBuff[3]  << 8);
	dig_T3 = (CalibBuff[4])  + (CalibBuff[5]  << 8);
	dig_P1 = (CalibBuff[6])  + (CalibBuff[7]  << 8);
	dig_P2 = (CalibBuff[8])  + (CalibBuff[9]  << 8);
	dig_P3 = (CalibBuff[10]) + (CalibBuff[11] << 8);
	dig_P4 = (CalibBuff[12]) + (CalibBuff[13] << 8);
	dig_P5 = (CalibBuff[14]) + (CalibBuff[15] << 8);
	dig_P6 = (CalibBuff[16]) + (CalibBuff[17] << 8);
	dig_P7 = (CalibBuff[18]) + (CalibBuff[19] << 8);
	dig_P8 = (CalibBuff[20]) + (CalibBuff[21] << 8);
	dig_P9 = (CalibBuff[22]) + (CalibBuff[23] << 8);

}

void BMP280_Get_Value(void){
	try:
	HAL_I2C_Mem_Read(&hi2c1 , BMP280_WRITE_REGISTER_ADDRESS , DataAddr , 1 , &Stage , 1 , 1000);

	if(Stage == 0x08 || Stage == 0x01 || Stage == 0x09){
		goto try;
	}

	/* Get values from the 0xF7 to 0xFC registers */
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_WRITE_REGISTER_ADDRESS , &DataAddr, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, BMP280_READ_REGISTER_ADDRESS , &RecvBuff[0], 6 , 1000);

	/* Bits are Drifting */
	/* Pressure Data Length is 20 Bit  */
	UnComp_Pressure_Array[0] = RecvBuff[0]; // Press_msb  -> 0xF7   -> 8 bit
	UnComp_Pressure_Array[1] = RecvBuff[1]; // Press_lsb  -> 0xF8   -> 8 bit
	UnComp_Pressure_Array[2] = RecvBuff[2]; // Press_xlsb -> 0xF9   -> 4 bit
	UnComp_Pressure = ((UnComp_Pressure_Array[0] << 12) + (UnComp_Pressure_Array[1] << 4) + (UnComp_Pressure_Array[2]));
	/* Temperature Data Length is 17 Bit  */
	UnComp_Temperature_Array[0] = RecvBuff[3]; // Temperature_msb -> 0xFA   -> 8 bit
	UnComp_Temperature_Array[1] = RecvBuff[4]; // Temperature_lsb -> 0xFB   -> 8 bit
	UnComp_Temperature_Array[2] = RecvBuff[5]; // Temperature_xlsb -> 0xFC  -> 1 bit
	UnComp_Temperature = ((UnComp_Temperature_Array[0] << 12) + (UnComp_Temperature_Array[1] << 4) + (UnComp_Temperature_Array[2]));


	//UnComp_Temperature = Moving_Average_Filter(UnComp_Temperature);

	/* Uncompensated Values converting to the Compensated Values */
	double var1, var2;
	var1=(((double)UnComp_Temperature)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	var2=((((double)UnComp_Temperature)/131072.0-((double)dig_T1)/8192.0)*(((double)UnComp_Temperature)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	double t_fine = (int32_t)(var1+var2);
	float T = (var1+var2)/5120.0;

	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)dig_P6)/32768.0;
	var2=var2+var1*((double)dig_P5)*2.0;
	var2=(var2/4.0)+(((double)dig_P4)*65536.0);
	var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)dig_P1);
	double p=1048576.0-(double)UnComp_Pressure;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)dig_P9)*p*p/2147483648.0;
	var2=p*((double)dig_P8)/32768.0;
	p=p+(var1+var2+((double)dig_P7))/16.0;

	temperature = T;
	pressure = p;
	altitude=44330.0f*(1-powf(p/101325.0f,1.0f/5.255f));
}

void Low_Pass_Filter_26Hz(void) {
	/* The Values Calculated In Matlab */
	Xn = UnComp_Pressure;
	Yn = 0.9838*Yn1 + 0.0162*Xn + 0.0162*Xn1;
	HAL_Delay(1);
	Xn1 = Xn;
	Yn1 = Yn;
	UnComp_Pressure = Yn;
}
