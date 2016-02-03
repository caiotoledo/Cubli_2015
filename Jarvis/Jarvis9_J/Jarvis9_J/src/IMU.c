/*
 * IMU.c
 *
 * Created: 20/11/2014 00:20:22
 *  Author: Caio
 */ 

#include <asf.h>
#include <math.h>
#include "IMU.h"

void config_IMU(void){
	if (twi_init() == TWI_SUCCESS){
		adxl_init();
		itg_init();
		} else {
		while(1){
			gpio_toggle_pin(LED2_GPIO);
			mdelay(5000);	//Delay 500.0 ms
		}
	}
}

uint8_t twi_init(void){
	/*TWI0:
	- Clock:	PA4 (Laranja)
	- Data:		PA3 (Cinza)
	- Terra:	GND (Preta)
	- VCC:		3V3 (Branco)*/
	gpio_configure_pin(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	gpio_configure_pin(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
	
	twi_options_t opt_twi;
	
	pmc_enable_periph_clk(ID_TWI0);	
	
	opt_twi.master_clk = sysclk_get_cpu_hz();
	opt_twi.speed = TWI_SPEED;
	opt_twi.smbus = 0;
	
	uint8_t twi_status = twi_master_init(TWI0, &opt_twi);
	
	return twi_status;
}

void adxl_init(void) 
{
	//Low Address:
	adxl_write(ADXL_ADDR_DATAFORMAT,0x0B,ADXL_ADDR_LOW);	//16-bit, 13-bit mode
	//adxl_write(ADXL_ADDR_BW_RATE,0x0F,ADXL_ADDR_LOW);		//Sample rate = 3200Hz = 0,3125ms LPF = 1600Hz
	//adxl_write(ADXL_ADDR_BW_RATE,0x0D,ADXL_ADDR_LOW);		//Sample rate = 800Hz = 1,25ms LPF = 400Hz
	//adxl_write(ADXL_ADDR_BW_RATE,0x0C,ADXL_ADDR_LOW);		//Sample rate = 400Hz = 2,5ms LPF = 200Hz
	//adxl_write(ADXL_ADDR_BW_RATE,0x0A,ADXL_ADDR_LOW);		//Sample rate = 100Hz = 10ms LPF = 50Hz
	adxl_write(ADXL_ADDR_BW_RATE,0x09,ADXL_ADDR_LOW);		//Sample rate = 50Hz = 20ms LPF = 25Hz
	adxl_write(ADXL_ADDR_POWERCTL,0x08,ADXL_ADDR_LOW);		//Start Measurement
	
	//High Address:
	adxl_write(ADXL_ADDR_DATAFORMAT,0x0B,ADXL_ADDR_HIGH);	//16-bit, 13-bit mode
	//adxl_write(ADXL_ADDR_BW_RATE,0x0F,ADXL_ADDR_HIGH);		//Sample rate = 3200Hz = 0,3125ms
	//adxl_write(ADXL_ADDR_BW_RATE,0x0D,ADXL_ADDR_HIGH);		//Sample rate = 800Hz = 1,25ms
	//adxl_write(ADXL_ADDR_BW_RATE,0x0C,ADXL_ADDR_HIGH);		//Sample rate = 400Hz = 2,5ms LPF = 200Hz
	//adxl_write(ADXL_ADDR_BW_RATE,0x0A,ADXL_ADDR_HIGH);		//Sample rate = 100Hz = 10ms LPF = 50Hz
	adxl_write(ADXL_ADDR_BW_RATE,0x09,ADXL_ADDR_LOW);		//Sample rate = 50Hz = 20ms LPF = 25Hz
	adxl_write(ADXL_ADDR_POWERCTL,0x08,ADXL_ADDR_HIGH);		//Start Measurement	
}

void itg_init(void)
{
	//Low Address:
	//itg_write(ITG_ADDR_DLPF_FS,0x18,ITG_ADDR_LOW);	//2000º/s - Sample Rate: 8KHz - LPF: 256Hz
	//itg_write(ITG_ADDR_DLPF_FS,0x1E,ITG_ADDR_LOW);	//2000º/s - Sample Rate: 1KHz - LPF: 5Hz
	itg_write(ITG_ADDR_DLPF_FS,0x1B,ITG_ADDR_LOW);	//2000º/s - Sample Rate: 1KHz - LPF: 42Hz
	itg_write(ITG_ADDR_SMPLRT_DIV, 0,ITG_ADDR_LOW);	//Fsample = 8KHz/(0 + 1) = 8KHz : 0,125ms
	itg_write(ITG_ADDR_INT_CFG,0x11,ITG_ADDR_LOW);	//Latch mode: 50us Pulse INT when data is ready - Clear INT when any register is read
	itg_write(ITG_ADDR_PWR_MGM,0x00,ITG_ADDR_LOW);	//Clock Source: Internal Oscillator
	
	//High Address:
	//itg_write(ITG_ADDR_DLPF_FS,0x18,ITG_ADDR_HIGH);	//2000º/s - Sample Rate: 8KHz - LPF: 256Hz
	//itg_write(ITG_ADDR_DLPF_FS,0x1E,ITG_ADDR_HIGH);	//2000º/s - Sample Rate: 1KHz - LPF: 5Hz
	itg_write(ITG_ADDR_DLPF_FS,0x1B,ITG_ADDR_HIGH);	//2000º/s - Sample Rate: 1KHz - LPF: 42Hz
	itg_write(ITG_ADDR_SMPLRT_DIV, 0,ITG_ADDR_HIGH);//Fsample = 8KHz/(0 + 1) = 8KHz : 0,125ms
	itg_write(ITG_ADDR_INT_CFG,0x11,ITG_ADDR_HIGH);	//Latch mode: 50us Pulse INT when data is ready - Clear INT when any register is read
	itg_write(ITG_ADDR_PWR_MGM,0x00,ITG_ADDR_HIGH);	//Clock Source: Internal Oscillator
}

uint32_t adxl_write(
				uint8_t index,
				uint8_t  value,
				uint8_t addr){
	twi_packet_t tx;
	
	tx.addr[0]		=	index;
	tx.addr_length	=	1;
	tx.buffer		=	&value;
	tx.length		=	1;
	tx.chip			=	addr;
	
	return	twi_master_write(TWI0, &tx);
}

uint32_t adxl_read(
				uint8_t index,
				uint8_t *value,
				uint8_t addr){
					
	twi_packet_t rx;
	
	rx.addr[0]		=	index;
	rx.addr_length	=	1;
	rx.buffer		=	value;
	rx.length		=	1;
	rx.chip			=	addr;
	
	return	twi_master_read(TWI0, &rx);
}

uint32_t itg_write(
				uint8_t index,
				uint8_t  value,
				uint8_t addr){
	twi_packet_t tx = {
		.addr[0]		= index,
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= addr
	};
	
	return	twi_master_write(TWI0, &tx);
}

uint32_t itg_read(
				uint8_t index,
				uint8_t *value,
				uint8_t addr){
	
	twi_packet_t rx = {
		.addr[0]		= index,
		.addr_length	= 1,
		.buffer			= value,
		.length			= 1,
		.chip			= addr
	};
	
	return	twi_master_read(TWI0, &rx);
}

uint32_t adxl_multiplereads(uint8_t index,
							uint8_t *value, 
							uint8_t length,
							uint8_t addr)
{
	
	twi_packet_t rx;
	
	rx.addr[0]		=	index;
	rx.addr_length	=	1;
	rx.buffer		=	value;
	rx.length		=	length;
	rx.chip			=	addr;
	
	return	twi_master_read(TWI0, &rx);
}

uint32_t itg_multiplereads(	uint8_t index,
							uint8_t *value,
							uint8_t length,
							uint8_t addr)
{
	
	twi_packet_t rx;
	
	rx.addr[0]		=	index;
	rx.addr_length	=	1;
	rx.buffer		=	value;
	rx.length		=	length;
	rx.chip			=	addr;
	
	return	twi_master_read(TWI0, &rx);
}

void get_all_gyro_value(float *value, uint8_t addr){
	static uint8_t buf[6];
	uint16_t itg = 0;
	
	
	itg_multiplereads(ITG_ADDR_DATAX1, &buf, 6, addr);
	
	//EIXO X:
	if ( (buf[0] & 0x80) == 0x80 ){
		itg = (buf[0] << 8) | (buf[1]);
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		value[0] = -(((float)itg) / 14.375);
	}
	else {
		itg = (buf[0] << 8) | (buf[1]);
		value[0] = (((float)itg) / 14.375);
	}
	
	//EIXO Y:
	if ( (buf[2] & 0x80) == 0x80 ){
		itg = (buf[2] << 8) | (buf[3]);
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		value[1] = -(((float)itg) / 14.375);
	}
	else {
		itg = (buf[2] << 8) | (buf[3]);
		value[1] = (((float)itg) / 14.375);
	}
	
	//EIXO Z:
	if ( (buf[4] & 0x80) == 0x80 ){
		itg = (buf[4] << 8) | (buf[5]);
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		value[2] = -(((float)itg) / 14.375);
	}
	else {
		itg = (buf[4] << 8) | (buf[5]);
		value[2] = (((float)itg) / 14.375);
	}
	
	//Offset de Eixos:
	if (addr == ITG_ADDR_HIGH){
		value[0] += ITG_OFSTX_HIGH;
		value[1] += ITG_OFSTY_HIGH;
		value[2] += ITG_OFSTZ_HIGH;
	}
	else if (addr == ITG_ADDR_LOW){
		value[0] += ITG_OFSTX_LOW;
		value[1] += ITG_OFSTY_LOW;
		value[2] += ITG_OFSTZ_LOW;
	}
}

uint8_t angleXY (float *value, uint8_t addr){
	static uint8_t buf[4];
	uint16_t adxl = 0;
	float value_X,value_Y = 0.0;
	
	uint32_t check = adxl_multiplereads(ADXL_ADDR_DATAX0, &buf, 4, addr);
	if (check != TWI_SUCCESS)
		return check;
	
	//EIXO X:
	if ( (buf[1] & 0xF0) == 0xF0 ){
		adxl = (buf[1] << 8) | (buf[0]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value_X = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[1] << 8) | (buf[0]);
		value_X = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//EIXO Y:
	if ( (buf[3] & 0xF0) == 0xF0 ){
		adxl = (buf[3] << 8) | (buf[2]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value_Y = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[3] << 8) | (buf[2]);
		value_Y = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//Offset dos Eixos:
	if (addr == ADXL_ADDR_HIGH){
		value_X = value_X + ADXL_OFSTX_HIGH;
		value_Y = value_Y + ADXL_OFSTY_HIGH;
	}
	else if (addr == ADXL_ADDR_LOW){
		value_X = value_X + ADXL_OFSTX_LOW;
		value_Y = value_Y + ADXL_OFSTY_LOW;
	}
	else {
		return check;
	}
	
	value_X = - value_X;				//Por conta de Posicionamento do IMU
	
	//Calculo do angulo:
	//*value = atanf(value_Y/value_X) * (180.0/PI);
	*value = asinf(value_X/value_Y) * (180.0/PI);
	
	return TWI_SUCCESS;
}

uint8_t angleXY_filter (float *angle_real, float *gyroData, uint8_t acel_addr, uint8_t gyro_addr){
	static uint8_t buf[4];
	
	/*
	AQUISIÇÃO DO ANGULO (ACELEROMETRO):
	*/
	uint16_t adxl = 0;
	float value_X,value_Y, angle_measure;
	
	uint32_t check = adxl_multiplereads(ADXL_ADDR_DATAX0, &buf, 4, acel_addr);
	if (check != TWI_SUCCESS)
	return check;
	//EIXO X:
	if ( (buf[1] & 0xF0) == 0xF0 ){
		adxl = (buf[1] << 8) | (buf[0]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value_X = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[1] << 8) | (buf[0]);
		value_X = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	//EIXO Y:
	if ( (buf[3] & 0xF0) == 0xF0 ){
		adxl = (buf[3] << 8) | (buf[2]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value_Y = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[3] << 8) | (buf[2]);
		value_Y = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	//Offset dos Eixos:
	if (acel_addr == ADXL_ADDR_HIGH){
		value_X = value_X + ADXL_OFSTX_HIGH;
		value_Y = value_Y + ADXL_OFSTY_HIGH;
	}
	else if (acel_addr == ADXL_ADDR_LOW){
		value_X = value_X + ADXL_OFSTX_LOW;
		value_Y = value_Y + ADXL_OFSTY_LOW;
	}
	else {
		return check;
	}
	
	value_X = - value_X;				//Por conta de Posicionamento do IMU
	
	//Calculo do Angulo pelos dados do acelerometro:
	//angle_measure = atanf(value_Y/value_X) * (180.0/PI);
	angle_measure = asinf(value_X/value_Y) * (180.0/PI);
	
	/*
	AQUISIÇÃO VELOCIDADE ANGULAR (GYRO):
	*/
	*gyroData = get_gyro_value('Z', gyro_addr);
	
	/*
	CALCULO FILTRO COMPLEMENTAR:
	angle_real: Estimated Angle
	gyroData: Rotation rate measured by gyro
	dt: Sample interval
	angle_measure: Angle calculated from the accelerometer measurements
	alpha: Constant
	beta: (1 - alpha)
	*/
	*angle_real = (*angle_real + *gyroData*dt)*alpha + beta*angle_measure;
	
	return TWI_SUCCESS;
}

void init_angle_dynamic(float *angle){
	for (uint32_t j = 0; j < SIZE_ANGLE; j++){
		angle[j] = 0;
		angle_measure[j] = 0;
	}
	flag_time_sample = 0;
	for ( uint32_t i = 0; i < 500; i++){
		flag_time_sample = 0;
		while(!flag_time_sample);
		angleXY_dynamic(angle);
	}
}

uint8_t angleXY_dynamic(float *angle){
	static uint8_t buf[4];
	uint16_t adxl = 0;
	float valueX_high,valueY_high,valueX_low,valueY_low;
	
	//ADDR HIGH:
	uint32_t check = adxl_multiplereads(ADXL_ADDR_DATAX0, &buf, 4, ADXL_ADDR_HIGH);
	if (check != TWI_SUCCESS)
		return check;
	//EIXO X:
	if ( (buf[1] & 0xF0) == 0xF0 ){
		adxl = (buf[1] << 8) | (buf[0]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		valueX_high = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[1] << 8) | (buf[0]);
		valueX_high = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	//EIXO Y:
	if ( (buf[3] & 0xF0) == 0xF0 ){
		adxl = (buf[3] << 8) | (buf[2]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		valueY_high = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[3] << 8) | (buf[2]);
		valueY_high = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//ADDR LOW:
	check = adxl_multiplereads(ADXL_ADDR_DATAX0, &buf, 4, ADXL_ADDR_LOW);
	if (check != TWI_SUCCESS)
	return check;
	//EIXO X:
	if ( (buf[1] & 0xF0) == 0xF0 ){
		adxl = (buf[1] << 8) | (buf[0]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		valueX_low = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[1] << 8) | (buf[0]);
		valueX_low = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	//EIXO Y:
	if ( (buf[3] & 0xF0) == 0xF0 ){
		adxl = (buf[3] << 8) | (buf[2]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		valueY_low = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[3] << 8) | (buf[2]);
		valueY_low = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//Offset dos Eixos:
	valueX_high = valueX_high + ADXL_OFSTX_HIGH;
	valueY_high = valueY_high + ADXL_OFSTY_HIGH;
	valueX_low = valueX_low + ADXL_OFSTX_LOW;
	valueY_low = valueY_low + ADXL_OFSTY_LOW;
	
	//ELIMINANDO FATOR DINAMICO:
	float mx, my;
	
	mx = valueX_high - U*(valueX_low);
	my = valueY_high - U*(valueY_low);
	
	shift_right(&angle_measure, SIZE_ANGLE);
	shift_right(angle, SIZE_ANGLE);
	angle_measure[0] = atanf(-mx/my);
	angle_measure[0] += DYNAMIC_OFFSET;
	
	angle[0] = I0*angle_measure[0] + I1*angle_measure[1] + O1*angle[1];
	
	return check;
}

void filter_lowpass(float *vetor, float *H, uint32_t size){
	float final = 0;
	for (uint32_t k = 0; k < size; k++){
		final += H[k]*vetor[k];
	}
	vetor[0] = final;
}

void shift_right (float *vetor, uint32_t size ){
	for (uint32_t k = (size-1); k > 0; k-- ){
		vetor[k] = vetor[k-1];
	}
}

void get_all_acel_value(float *value, uint8_t addr){
	static uint8_t buf[6];
	uint16_t adxl = 0;
	
	adxl_multiplereads(ADXL_ADDR_DATAX0, &buf, 6, addr);
	
	//EIXO X:
	if ( (buf[1] & 0xF0) == 0xF0 ){
		adxl = (buf[1] << 8) | (buf[0]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value[0] = -(ADXL_SCALE_FACTOR * ((float)adxl));
	} 
	else {
		adxl = (buf[1] << 8) | (buf[0]);
		value[0] = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//EIXO Y:
	if ( (buf[3] & 0xF0) == 0xF0 ){
		adxl = (buf[3] << 8) | (buf[2]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value[1] = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[3] << 8) | (buf[2]);
		value[1] = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//EIXO Z:
	if ( (buf[5] & 0xF0) == 0xF0 ){
		adxl = (buf[5] << 8) | (buf[4]);
		adxl = ( ( (~adxl) +1 ) & 0x0FFF);
		value[2] = -(ADXL_SCALE_FACTOR * ((float)adxl));
	}
	else {
		adxl = (buf[5] << 8) | (buf[4]);
		value[2] = (ADXL_SCALE_FACTOR * ((float)adxl));
	}
	
	//Offset dos Eixos:
	if (addr == ADXL_ADDR_HIGH){
		value[0] += ADXL_OFSTX_HIGH;
		value[1] += ADXL_OFSTY_HIGH;
		value[2] += ADXL_OFSTZ_HIGH;
	}
	else if (addr == ADXL_ADDR_LOW){
		value[0] += ADXL_OFSTX_LOW;
		value[1] += ADXL_OFSTY_LOW;
		value[2] += ADXL_OFSTZ_LOW;
	}
}

float get_gyro_value(char eixo, uint8_t addr){
	float valor = 0;
	uint16_t itg = 0;
	static uint8_t buf = 0;
	
	switch (eixo){
		case 'X':
		itg_read(ITG_ADDR_DATAX0, &buf, addr);
		itg |= buf;
		itg_read(ITG_ADDR_DATAX1, &buf, addr);
		break;
		case 'Y':
		itg_read(ITG_ADDR_DATAY0, &buf, addr);
		itg |= buf;
		itg_read(ITG_ADDR_DATAY1, &buf, addr);
		break;
		case 'Z':
		itg_read(ITG_ADDR_DATAZ0, &buf, addr);
		itg |= buf;
		itg_read(ITG_ADDR_DATAZ1, &buf, addr);
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
	
	if (eixo == 'Z'){
		if (addr == ITG_ADDR_LOW){
			valor += ITG_OFSTZ_LOW;
		} 
		else if (addr == ITG_ADDR_HIGH){
			valor += ITG_OFSTZ_HIGH;
		}
	}
	
	valor = (-valor*DegreesToRad);
	
	return valor;
}

float get_acel_value(char eixo, uint8_t addr){
	float valor = 0;
	uint16_t adxl = 0;
	static uint8_t buf = 0;
	
	switch (eixo){
		case 'X':
		adxl_read(ADXL_ADDR_DATAX0, &buf, addr);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAX1, &buf, addr);
		break;
		case 'Y':
		adxl_read(ADXL_ADDR_DATAY0, &buf, addr);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAY1, &buf, addr);
		break;
		case 'Z':
		adxl_read(ADXL_ADDR_DATAZ0, &buf, addr);
		adxl = buf;
		adxl_read(ADXL_ADDR_DATAZ1, &buf, addr);
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

void clear_interrupt (uint8_t addr){
	uint8_t read;
	adxl_read(ADXL_ADDR_DATAX0, &read, addr);
	itg_read(ITG_ADDR_DATAX0, &read, addr);
}