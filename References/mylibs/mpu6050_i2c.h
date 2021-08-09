#include "stm32f3xx_hal.h"
#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H
#define DPS_2000 24U
#define DPS_1000 16U
#define DPS_500 08U
#define DPS_250 00U
#define LPF_BEST 06U
#define LPF_MEDIUM 04U
#define LPF_FAIR 02U
#define LPF_NO 00U
const uint8_t mpu9250 = (0x68<<1);
typedef struct gyro_init{
	uint8_t dps;
	uint8_t LPF;
	uint8_t SMPL_DIV;
	
}gyro_init;
typedef struct gyroraw
{
    int16_t x;
    int16_t y;
    int16_t z;
}gyroraw;
typedef struct gyrodps{
	float x;
	float y;
	float z;
}gyrodps;


#endif