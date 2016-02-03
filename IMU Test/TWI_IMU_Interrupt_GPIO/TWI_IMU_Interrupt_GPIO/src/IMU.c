/*
 * IMU.c
 *
 * Created: 20/11/2014 00:20:22
 *  Author: Caio
 */ 

#include <asf.h>
#include "IMU.h"

void twi_init(void){
	twi_options_t opt_twi;
	
	// disable JTAG
	//MATRIX->CCFG_SYSIO |= (1 << 4) | (1 << 5);
	/*REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
	REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO5;*/
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
			gpio_set_pin_low(LED2_GPIO);
		}
	}
	
	pmc_enable_periph_clk(ID_TWI0);
	
	opt_twi.master_clk = sysclk_get_cpu_hz();
	opt_twi.speed = TWI_SPEED;
	opt_twi.smbus = 0;
	
	if (twi_master_init(TWI0, &opt_twi) != TWI_SUCCESS) {
		puts("-E-\tTWI master initialization failed.\r");
		while (1) {
			/* Capture error */
			gpio_set_pin_low(LED2_GPIO);
		}
	}
}

void adxl_init(void) 
{
	adxl_write(ADXL_ADDR_DATAFORMAT,0x0B);	//16-bit, 13-bit mode
	//adxl_write(ADXL_ADDR_BW_RATE,0x07);		//Sample rate = 12,5Hz = 80ms
	adxl_write(ADXL_ADDR_BW_RATE,0x06);		//Sample rate = 6,25Hz = 160ms
	adxl_write(ADXL_ADDR_INT_ENABLE,0x80);	//DATA-READY Interrupt
	adxl_write(ADXL_ADDR_INT_MAP,0x7F);		//DATA-READY Interrupt - INT1 / Rest - INT2
	adxl_write(ADXL_ADDR_POWERCTL,0x08);	//Start Measurement
	
	//Configurando Offset:
	adxl_write(ADXL_ADDR_OFSTY,0x20);		// 0x20 * 15,6[mg/LSB] = 499,2 mg
	adxl_write(ADXL_ADDR_OFSTX,0x13);		// 0x13 * 15,6[mg/LSB] = 296,4 mg
	adxl_write(ADXL_ADDR_OFSTZ,0x81);		// 0x81 * 15,6[mg/LSB] = -3962,4 mg
}

void itg_init(void)
{
	itg_write(ITG_ADDR_DLPF_FS,0x1B);	//2000º/s - Sample Rate: 1KHz - LPF: 42Hz
	itg_write(ITG_ADDR_SMPLRT_DIV, 99);	//Fsample = 1KHz/(99 + 1) = 10Hz : 100ms
	itg_write(ITG_ADDR_INT_CFG,0x11);	//Latch mode: 50us Pulse INT when data is ready - Clear INT when any register is read
	//itg_write(ITG_ADDR_INT_CFG,0x31);	//Latch mode: High State INT when data is ready - Clear INT when any register is read
	itg_write(ITG_ADDR_PWR_MGM,0x00);	//Clock Source: Internal Oscillator
}

uint32_t adxl_write(
				uint8_t index,
				uint8_t  value){
	twi_packet_t tx;
	
	tx.addr[0]		=	index;
	tx.addr_length	=	1;
	tx.buffer		=	&value;
	tx.length		=	1;
	tx.chip			=	ADXL_ADDR;
	
	return	twi_master_write(TWI0, &tx);
}

uint32_t adxl_read(
				uint8_t index,
				uint8_t *value){
					
	twi_packet_t rx;
	
	rx.addr[0]		=	index;
	rx.addr_length	=	1;
	rx.buffer		=	value;
	rx.length		=	1;
	rx.chip			=	ADXL_ADDR;
	
	return	twi_master_read(TWI0, &rx);
}

uint32_t itg_write(
				uint8_t index,
				uint8_t  value){
	twi_packet_t tx = {
		.addr[0]		= index,
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ITG_ADDR
	};
	
	return	twi_master_write(TWI0, &tx);
}

uint32_t itg_read(
				uint8_t index,
				uint8_t *value){
	
	twi_packet_t rx = {
		.addr[0]		= index,
		.addr_length	= 1,
		.buffer			= value,
		.length			= 1,
		.chip			= ITG_ADDR
	};
	
	return	twi_master_read(TWI0, &rx);
}

float get_gyro_value(char eixo){
	float valor = 0;
	uint16_t itg = 0;
	static uint8_t buf = 0;
	
	switch (eixo){
		case 'X':
		itg_read(ITG_ADDR_DATAX0, &buf);
		itg |= buf;
		itg_read(ITG_ADDR_DATAX1, &buf);
		break;
		case 'Y':
		itg_read(ITG_ADDR_DATAY0, &buf);
		itg |= buf;
		itg_read(ITG_ADDR_DATAY1, &buf);
		break;
		case 'Z':
		itg_read(ITG_ADDR_DATAZ0, &buf);
		itg |= buf;
		itg_read(ITG_ADDR_DATAZ1, &buf);
		break;
	}
	
	if ((buf & 0x80) == 0x80){
		itg |= (buf << 8);
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		valor = -(((float)itg) / 14.375);
		} else {
		itg |= ( buf << 8);
		valor = ((float)itg) / 14.375;
	}
	
	return valor;
}

float get_acel_value(char eixo){
	float valor = 0;
	uint16_t adxl = 0;
	static uint8_t buf = 0;
	
	switch (eixo){
		case 'X':
		adxl_read(ADXL_ADDR_DATAX0, &buf);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAX1, &buf);
		break;
		case 'Y':
		adxl_read(ADXL_ADDR_DATAY0, &buf);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAY1, &buf);
		break;
		case 'Z':
		adxl_read(ADXL_ADDR_DATAZ0, &buf);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAZ1, &buf);
		break;
	}
	
	if ((buf & 0xF0) == 0xF0){
		adxl |= (buf << 8);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		valor = -(3.9 * ((float)adxl));
		//adxl_global = - (int16_t)(adxl);
		} else {
		adxl |= ( buf << 8);
		valor = 3.9 * ((float)adxl);
		//adxl_global = (int16_t)(adxl);
	}
	
	return valor;
}