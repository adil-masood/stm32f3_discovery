#include "mpu6050_i2c.h"
#include "stm32f3xx_hal.h"
#include "string.h"
//#include "stm32f3xx_hal_i2c.h"
#define DPS_2000 24U
#define DPS_1000 16U
#define DPS_500 08U
#define DPS_250 00U
#define LPF_BEST 06U
#define LPF_MEDIUM 04U
#define LPF_FAIR 02U
#define LPF_NO 00U

// 1st pointer to register at address 0x19
  // 2nd addr=0x19 <SMPLRT_DIV registr> || "SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)"
  // 3rd 00000110 addr=0x1A <CONFIG registr> || "DLPF <Fs=1KHz, mode=6>, fifo mode replace"
  // 4th 1000dps addr=0x1B <GYRO CONFIG registr> || "gyro full scale +-250dps, enable DLPF by fchoice_b = 2b'00"
gyro_init gyro_new(uint8_t DPS,uint8_t LPF,uint8_t sample_div){
	gyro_init any = {DPS, LPF, sample_div};
	return any;
}
	
gyro_init gyroinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,gyro_init *self){
	//smpl_address,smpl_div,conf,gyro_conf
	uint8_t init[4] = {0x19,self->SMPL_DIV,self->LPF,self->dps};
	//int pin configs
	//uint8_t buff2[3] = {0x37,0x10,0x01};
	if(HAL_I2C_Master_Transmit(hi2c,mpu9250,init,4,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (1)",strlen("i2c_error: gyro_init() (1)"),HAL_MAX_DELAY);
	}
}
gyroraw get_gyroraw(I2C_HandleTypeDef *hi2c,gyroraw *self){
	uint8_t gyro_add = 0x43;
	uint8_t gyroval[6];
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&gyro_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,gyroval,6,HAL_MAX_DELAY);
	self->x = ((int16_t)((gyroval[0]<<8) | gyroval[1]));
	self->y = ((int16_t)((gyroval[2]<<8) | gyroval[3]));
	self->z = ((int16_t)((gyroval[4]<<8) | gyroval[5]));
}
gyrodps to_dps(I2C_HandleTypeDef *hi2c,gyroraw *self){
	gyrodps gyro_dps;
	gyro_dps.x= ((float)self->x)/100;
	gyro_dps.x= ((float)((int)self->x))*100;
	gyro_dps.x= (((1000)/32768.)*(self->x)*(0.02));
	gyro_dps.y= ((float)self->y)/100;
	gyro_dps.y= ((float)((int)self->y))*100;
	gyro_dps.y= (((1000)/32768.)*(self->y)*(0.02));
	gyro_dps.z= ((float)self->z)/100;
	gyro_dps.z= ((float)((int)self->z))*100;
	gyro_dps.z= (((1000)/32768.)*(self->z)*(0.02));
	return gyro_dps;
}
void get_status(I2C_HandleTypeDef *hi2c,uint8_t *status){
	uint8_t status_add = 0x3A;
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&status_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,status,1,HAL_MAX_DELAY);
}
