/*
 * Motor.c
 *
 * Created: 14/09/2015 18:48:27
 *  Author: Caio
 */ 

#include <asf.h>
#include "Motor.h"
#include <math.h>

static const uint32_t divisors[5] = {2, 8, 32, 128, 0};

//static uint32_t cont_pulses = 0;
static uint32_t capture_rb_freq = 0;
static uint32_t capture_ra_duty = 0;
static uint32_t capture_rb_duty = 0;

const float rpstorpm = 60; 
volatile int32_t dac_val = 0;

static uint8_t flag_revertion = 1;
static uint8_t flag_neg = 0;
//static uint8_t flag_hall = 0;
static uint8_t hallA = 0;
static uint8_t hallB = 0;

volatile uint8_t flag_speed = SPEED_DUTY;

void config_motor(void){
	/*DACC = PB13*/
	config_dacc();
	update_speed_motor(0);
	/*TC_Freq = PA26*/
	config_tccapture_freq();
	/*TC_Duty = PC23*/
	config_tccapture_duty();
	
	/*GPIO_HALLB = PA20
	  GPIO_HALLC = PA21*/
	config_hall_interrupt();
	
	//CONFIGURANDO PORTA OPEN-DRAIN:
	gpio_configure_pin( GPIO_FR , GPIO_FR_conf );
	Foward_Motor;
}

void update_speed_motor(float speed){
	uint32_t negativo = 0;
	//Verificando a necessidade de alterar a rotação do motor:
	if ( (speed < 0) && (MotorIsFoward) ){
		Reverse_Motor; //Invertendo rotação do Motor
		//speed = -speed;
		flag_revertion = !flag_revertion;
		hallA = 0;
		hallB = 0;
		} else if ( (speed > 0) && (MotorIsReverse) ) {
		Foward_Motor;
		flag_revertion = !flag_revertion;
		hallA = 0;
		hallB = 0;
	}
	
	if (speed < 0){
		speed = -speed;
		negativo = 1;
	}
	
	if (speed > MOTOR_MAX_RPM){
		speed = MOTOR_MAX_RPM;
	}
	//Transformar Valor de RPM para DAC(0-1023)
	dac_val = speed*RPMtoDAC + yo;
	while((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY);
	dacc_write_conversion_data(DACC, dac_val);
	if (negativo){
		dac_val = -dac_val;
	}
}

void update_speed_motor_dac(float dac_float){
	uint32_t negativo = 0;
	//Verificando a necessidade de alterar a rotação do motor:
	if ( (dac_float < 0) && (MotorIsFoward) ){
		Reverse_Motor; //Invertendo rotação do Motor
		flag_revertion = !flag_revertion;
		hallA = 0;
		hallB = 0;
		} else if ( (dac_float > 0) && (MotorIsReverse) ) {
		Foward_Motor;
		flag_revertion = !flag_revertion;
		hallA = 0;
		hallB = 0;
	}
	
	if (dac_float < 0){
		dac_float = -dac_float;
		negativo = 1;
	}
	
	if (dac_float > DACC_MAX_DATA){
		dac_float = DACC_MAX_DATA;
	}
	//Transformar Valor de RPM para DAC(0-1023)
	dac_val = dac_float;
	while((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY);
	dacc_write_conversion_data(DACC, dac_val);
	if (negativo){
		dac_val = -dac_val;
	}
}

float update_accel_motor(float accel, float last_speed, float t_seconds){
	// v[RPM] = vo[RPM] + a[RPM/s]*t[s]
	float new_speed = (last_speed + accel*t_seconds);
	update_speed_motor(new_speed);
	return new_speed;
}

void init_measure_speed(float *speed, uint32_t size){
	for (uint32_t j = 0; j < size; j++){
		speed[j] = 0;
	}
	flag_speed = SPEED_DUTY;
}

void measure_speed (float *speed, uint32_t size){
	shift_right(speed,size);	//Abrindo Espaço na Fila
	
	//Medição de velocidade pela Duty:
	if (flag_speed == SPEED_DUTY){
		measure_speed_duty(speed);
		if (speed[0] > MAX_DUTY){
			flag_speed = SPEED_FREQ;
		}
	} 
	//Medição de velocidade pela Freq:
	else if (flag_speed == SPEED_FREQ){
		measure_speed_freq(speed);
		if (speed[0] < MIN_FREQ){
			flag_speed = SPEED_DUTY;
		}
	}
	
	speed[0] = speed[0]*RPMtoRADs;	//Convertendo RPM para Rad/s
	filter_lowpass(speed,&Hd,size);
	//speed[0] = N0*speed[0] + N1*speed[1];
}

void measure_speed_freq(float *s_freq){
	// Em RPM:
	if (capture_rb_freq != 0){
		*s_freq = ((float)((float)(sysclk_get_peripheral_bus_hz(TC0) / divisors[TC_CAPTURE_TCCLKS])) / (capture_rb_freq))*RPStoRPM;
		capture_rb_freq = 0;
		if (flag_neg){
			*s_freq = - *s_freq;
		}
	} else {
		*s_freq = 0;
	}
}

void measure_speed_duty(float *s_duty){
	// Definir Fator_Duty
	if ( (capture_rb_duty != 0) && (capture_ra_duty != 0) ){
		*s_duty = ((1-((float)capture_ra_duty/capture_rb_duty))*100)*Fator_Duty;
		capture_rb_duty = 0;
		capture_ra_duty = 0;
		if (flag_neg){
			*s_duty = -*s_duty;
		}
	} else {
		*s_duty = 0;
	}
}

void config_hall_interrupt (void){
	pmc_enable_periph_clk(ID_HALL);
	
	pio_set_input(PIOA, GPIO_HALLB, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_HALLB, ENABLE);
	pio_handler_set(PIOA, ID_HALL, GPIO_HALLB, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA, GPIO_HALLB);
	
	pio_set_input(PIOA, GPIO_HALLC, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_HALLC, ENABLE);
	pio_handler_set(PIOA, ID_HALL, GPIO_HALLC, PIO_IT_FALL_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA, GPIO_HALLC);
	
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, PRIORITY_MEASURE);
	NVIC_EnableIRQ(PIOA_IRQn);
}

void config_dacc(void){
	sysclk_enable_peripheral_clock(ID_DACC);
	/* Reset DACC registers */
	dacc_reset(DACC);
	/* Half-Word transfer mode */
	dacc_set_transfer_mode(DACC, 0);
	/* Timing:
	 * startup                - 0x10 (17 clocks)
	 * internal trigger clock - 0x60 (96 clocks)
	 */
	dacc_set_timing(DACC, 0x10, 0x60);
	/*External trigger mode disabled. DACC in free running mode.*/
	dacc_disable_trigger(DACC);
	/* Enable DAC */
	dacc_enable(DACC);
	
	while((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY);
	dacc_write_conversion_data(DACC, 0);
}

void config_tccapture_freq(void){
	/** Configure PIO Pins for TC */
	ioport_set_pin_mode(PIN_TC0_TIOA2, PIN_TC0_TIOA2_MUX );
	/** Disable I/O to enable peripheral mode) */
	ioport_disable_pin(PIN_TC0_TIOA2);
	
	sysclk_enable_peripheral_clock(ID_TC2);
	
	tc_init(TC0, TC_CHANNEL_CAP_FREQ,
	TC_CAPTURE_TCCLKS			|	// Clock Selection = MCLK/32 CLK3
	TC_CMR_LDRA_RISING			|	// RA Loading = Rising edge of TIOA
	TC_CMR_LDRB_FALLING			|	// RB Loading = Falling Edge of TIOA
	TC_CMR_ABETRG				|	// External Trigger = TIOA
	TC_CMR_ETRGEDG_FALLING			// External Trigger Edge = Falling Edge
	);
	
	NVIC_DisableIRQ(TC2_IRQn);
	NVIC_ClearPendingIRQ(TC2_IRQn);
	NVIC_SetPriority(TC2_IRQn, PRIORITY_MEASURE);
	NVIC_EnableIRQ(TC2_IRQn);
	
	tc_enable_interrupt(TC0, TC_CHANNEL_CAP_FREQ, TC_IER_LDRBS);
	tc_start(TC0, TC_CHANNEL_CAP_FREQ);
}

void config_tccapture_duty(void){
	/** Configure PIO Pins for TC */
	ioport_set_pin_mode(PIO_PC23_IDX, IOPORT_MODE_MUX_B );
	/** Disable I/O to enable peripheral mode) */
	ioport_disable_pin(PIO_PC23_IDX);
	
	sysclk_enable_peripheral_clock(ID_TC3);
	
	tc_init(TC1, TC_CHANNEL_CAP_DUTY,
	TC_CAP_DUTY_TCCLKS			|	// Clock Selection = MCLK/32
	TC_CMR_LDRA_RISING			|	// RA Loading = Rising edge of TIOA
	TC_CMR_LDRB_FALLING			|	// RB Loading = Falling Edge of TIOA
	TC_CMR_ABETRG				|	// External Trigger = TIOA
	TC_CMR_ETRGEDG_FALLING			// External Trigger Edge = Falling Edge
	);
	
	NVIC_DisableIRQ(TC3_IRQn);
	NVIC_ClearPendingIRQ(TC3_IRQn);
	NVIC_SetPriority(TC3_IRQn, PRIORITY_MEASURE);
	NVIC_EnableIRQ(TC3_IRQn);
	
	tc_enable_interrupt(TC1, TC_CHANNEL_CAP_DUTY, TC_IER_LDRBS);
	tc_start(TC1, TC_CHANNEL_CAP_DUTY);
}

void pin_riseedge_handler(uint32_t id, uint32_t mask){
	if ( (id == ID_HALL) && ( mask == GPIO_HALLB ) ){		
		hallB = 1;
		if (hallA){
			flag_neg = 1;
		}
	}
	if ( (id == ID_HALL) && ( mask == GPIO_HALLC ) ){		
		hallA = 0;
		hallB = 0;
	}
}

void TC_FREQ_HANDLER(void){
	if ((tc_get_status(TC0, TC_CHANNEL_CAP_FREQ) & TC_SR_LDRBS) == TC_SR_LDRBS) {
		capture_rb_freq = tc_read_rb(TC0, TC_CHANNEL_CAP_FREQ);
		
		hallA = 1;
		if (hallB){
			flag_neg = 0;
		}
	}
}

void TC_DUTY_HANDLER(void){
	if ((tc_get_status(TC1, TC_CHANNEL_CAP_DUTY) & TC_SR_LDRBS) == TC_SR_LDRBS) {
		capture_ra_duty = tc_read_ra(TC1, TC_CHANNEL_CAP_DUTY);
		capture_rb_duty = tc_read_rb(TC1, TC_CHANNEL_CAP_DUTY);
	}
}