/*
 * Motor.h
 *
 * Created: 14/09/2015 18:47:57
 *  Author: Caio
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "IMU.h"
#include "Timer.h"

#define RPMtoRADs			(PI/30)
#define RADstoRPM			(30/PI)

#define SPEED_INI			0

#define DACC_MAX_VOLTAGE	3.3
#define GAIN				1.1151	//3.68V de pico

#define GPIO_FR				PIO_PA2_IDX
#define GPIO_FR_conf		(PIO_OPENDRAIN | PIO_OUTPUT_1)
#define Foward_Motor		gpio_set_pin_low(GPIO_FR)
#define Reverse_Motor		gpio_set_pin_high(GPIO_FR)
#define MotorIsReverse		gpio_pin_is_high(GPIO_FR)
#define MotorIsFoward		gpio_pin_is_low(GPIO_FR)

//PINOS DO HALL: (UTILIZANDO O TC_FREQ PARA HALL_A)
#define GPIO_HALLB			PIO_PA20
#define GPIO_HALLC			PIO_PA21
#define ID_HALL				ID_PIOA

//CONSTANTES DO MOTOR:
#define MOTOR_MAX_RPM		7563.4688
#define RPMtoDAC			0.14//DACC_MAX_DATA/MOTOR_MAX_RPM //0.1353//0.3846
#define DEAD_ZONE			276.42
#define yo					0.0//37.424

//TIMER-COUNTER: CAPTURE FREQUENCY
#define TC_CHANNEL_CAP_FREQ	2
#define TC_CAPTURE_TCCLKS	TC_CMR_TCCLKS_TIMER_CLOCK3	//Clock Amostragem Capture
#define TC_FREQ_HANDLER		TC2_Handler
#define RPStoRPM			60/8

//TIMER-COUNTER: CAPTURE DUTY
#define TC_CHANNEL_CAP_DUTY	0
#define TC_CAP_DUTY_TCCLKS	TC_CMR_TCCLKS_TIMER_CLOCK3	//Clock Amostragem Capture
#define TC_DUTY_HANDLER		TC3_Handler
#define Fator_Duty			80//81.15						//a Definir - Anterior 87.
#define Zona_Derivada		250							//Utilizado para identificar ponto de inflexão

//Metodo de Medição de Velocidade
#define SPEED_DUTY			0
#define SPEED_FREQ			1
#define MAX_DUTY			600
#define MIN_FREQ			500

void config_motor(void);
void update_speed_motor(float speed);
void update_speed_motor_dac(float dac_float);
float update_accel_motor(float accel, float last_speed, float t_seconds);
void init_measure_speed(float *speed, uint32_t size);
void measure_speed (float *speed, uint32_t size);
void measure_speed_freq(float *s_freq);
void measure_speed_duty(float *s_duty);
void pin_riseedge_handler(uint32_t id, uint32_t mask);
void config_hall_interrupt (void);
void config_dacc(void);
void config_tccapture_freq(void);
void config_tccapture_duty(void);

extern volatile uint8_t flag_speed;
extern volatile int32_t dac_val;

#endif /* MOTOR_H_ */