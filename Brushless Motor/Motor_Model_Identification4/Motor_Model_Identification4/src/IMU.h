/*
 * IMU.h
 *
 * Created: 20/11/2014 00:23:34
 *  Author: Caio
 */ 


#ifndef IMU_H_
#define IMU_H_

#include "Timer.h"

//#define TIMER_SAMPLE			20		//#define realizado na biblioteca Timer.h
#define PI						3.141592654
//#define RadToDegrees			(180.0/PI)
#define DegreesToRad			(PI/180.0)

#define TWI_SPEED				400000	//Fast-speed

#define READ_OK					1

#define ADXL_OFSTX_LOW			(- 300.0)//(- 500.0)
#define ADXL_OFSTY_LOW			(+ 490.0)//(+ 700.0)
#define ADXL_OFSTZ_LOW			(- 5000.0)

#define ADXL_OFSTX_HIGH			(+ 180.0)//(- 70.0)
#define ADXL_OFSTY_HIGH			(+ 654.0)//(+ 500.0)
#define ADXL_OFSTZ_HIGH			(- 5400.0)

#define ADXL_SCALE_FACTOR		3.9

#define ADXL_ADDR_LOW			0x53	//Endereço Padrão
#define ADXL_ADDR_HIGH			0x1D	//Endereço Modificado
#define ADXL_ADDR_DATAFORMAT	0X31
#define ADXL_ADDR_POWERCTL		0X2D
#define ADXL_ADDR_INT_ENABLE	0X2E
#define ADXL_ADDR_DATAX0		0X32
#define ADXL_ADDR_DATAX1		0X33
#define ADXL_ADDR_DATAY0		0X34
#define ADXL_ADDR_DATAY1		0X35
#define ADXL_ADDR_DATAZ0		0X36
#define ADXL_ADDR_DATAZ1		0X37
#define ADXL_ADDR_OFSTX			0X1E
#define ADXL_ADDR_OFSTY			0X1F
#define ADXL_ADDR_OFSTZ			0X20
#define ADXL_ADDR_BW_RATE		0x2C
#define ADXL_ADDR_INT_MAP		0x2F

#define ITG_OFSTX_LOW			(+ 0.0)
#define ITG_OFSTY_LOW			(+ 0.0)
#define ITG_OFSTZ_LOW			(- 0.56)

#define ITG_OFSTX_HIGH			(+ 0.0)
#define ITG_OFSTY_HIGH			(+ 0.0)
#define ITG_OFSTZ_HIGH			(+ 0.0)

#define DYNAMIC_OFFSET			(-0.017)//(+ 1.0)
#define N0						(0.12)
#define N1						(1 - N0)

#define ITG_ADDR_LOW			0x68	//Endereço Padrão
#define ITG_ADDR_HIGH			0x69	//Endereço Modificado
#define ITG_ADDR_SMPLRT_DIV		0x15
#define ITG_ADDR_DLPF_FS		0x16
#define ITG_ADDR_INT_CFG		0x17
#define ITG_ADDR_INT_STATUS		0x1A
#define ITG_ADDR_PWR_MGM		0x3E
#define ITG_ADDR_DATAX1			0X1D
#define ITG_ADDR_DATAX0			0X1E
#define ITG_ADDR_DATAY1			0X1F
#define ITG_ADDR_DATAY0			0X20
#define ITG_ADDR_DATAZ1			0X21
#define ITG_ADDR_DATAZ0			0X22

#define INT_ADXL				PIO_PA17
#define INT_ITG					PIO_PA18

//Relação R1/R2:
#define R1						15.5
#define R2						5.5

static const float U = R1/R2;

//CONSTANTES FILTRO COMPLEMENTAR:
#define DT						(TIME_SAMPLE/TIMER_CONSTANT)
#define ALPHA					0.7143//0.8333//0.9091
#define T_filter				1

static const float dt = (TIME_SAMPLE/TIMER_CONSTANT);
static const float alpha = ALPHA;
static const float beta = (1-ALPHA);

void config_IMU(void);
uint8_t twi_init(void);
void adxl_init(void);
void itg_init(void);
uint32_t adxl_write(uint8_t index,uint8_t value,uint8_t addr);
uint32_t adxl_read(uint8_t index,uint8_t *value,uint8_t addr);
uint32_t itg_write(uint8_t index,uint8_t  value,uint8_t addr);
uint32_t itg_read(uint8_t index,uint8_t *value,uint8_t addr);
uint32_t adxl_multiplereads(uint8_t index, uint8_t *value, uint8_t length,uint8_t addr);
uint32_t itg_multiplereads(uint8_t index, uint8_t *value, uint8_t length,uint8_t addr);
uint8_t angleXY (float *value, uint8_t addr);
uint8_t angleXY_filter (float *angle_real, float *gyroData, uint8_t acel_addr, uint8_t gyro_addr);
uint8_t angleXY_dynamic(float *angle, uint32_t size);
void init_angle_dynamic(float *angle, uint32_t size);
void get_all_acel_value(float *value,uint8_t addr);
void get_all_gyro_value(float *value,uint8_t addr);
float get_acel_value(char eixo,uint8_t addr);
float get_gyro_value(char eixo,uint8_t addr);
void clear_interrupt (uint8_t addr);

//int16_t adxl_global = 0;

#endif /* IMU_H_ */