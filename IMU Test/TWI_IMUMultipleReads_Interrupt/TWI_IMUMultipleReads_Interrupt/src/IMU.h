/*
 * IMU.h
 *
 * Created: 20/11/2014 00:23:34
 *  Author: Caio
 */ 


#ifndef IMU_H_
#define IMU_H_

#define TWI_SPEED				100000	//Fast-speed

#define ADXL_ADDR				0x53
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
#define ADXL_ADDR_BW_RATE		0x2C	//CONFIGURAR!
#define ADXL_ADDR_INT_MAP		0x2F	//CONFIGURAR!

#define ITG_ADDR				0x68
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

#define INT_ADXL				PIO_PA17 //Antes era PIO_PA16
#define INT_ITG					PIO_PA18 //Antes era PIO_PA10

void twi_init(void);
void adxl_init(void);
void itg_init(void);
uint32_t adxl_write(uint8_t index,uint8_t value);
uint32_t adxl_read(uint8_t index,uint8_t *value);
uint32_t itg_write(uint8_t index,uint8_t  value);
uint32_t itg_read(uint8_t index,uint8_t *value);
uint32_t adxl_multiplereads(uint8_t index, uint8_t *value, uint8_t length);
uint32_t itg_multiplereads(uint8_t index, uint8_t *value, uint8_t length);
void get_all_acel_value(float *value);
void get_all_gyro_value(float *value);
float get_acel_value(char eixo);
float get_gyro_value(char eixo);
void clear_interrupt (void);

//int16_t adxl_global = 0;

#endif /* IMU_H_ */