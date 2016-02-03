/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <math.h>
#include "Motor.h"
#include "Timer.h"
#include "UART_Comm.h"

#define SPEED_FINAL_INI		3000
#define RAMP_INI			1.5
#define CONSTANT_INI		3

void print_variables (void);
void rampa (void);
void step_variavel(void);
void step_simple(void);
void receive_dac(void);

float constant = CONSTANT_INI;
float ramp = RAMP_INI;
float speed_final = SPEED_FINAL_INI;
char buf[200];

float f;
float d[] = { 0, 0, 0 };
const float t_seconds = ((float)TIME_SAMPLE/TIMER_CONSTANT);
float speed = SPEED_INI;
float acceleration = 0;
uint32_t ramp_timer = 0;
uint32_t constant_timer = 0;
uint8_t flag_go = 0;
uint32_t cont_timer = 0;

uint32_t ramp_timer_second = 0;
uint32_t constant_timer_second = 0;
uint32_t final_time = 0;

float passo_step = 150.0;
unsigned int divisions = 2;
uint32_t par_divisor = 0;
uint32_t impar_mult = 0;
uint32_t step_timer = 0;
uint32_t cont_step = 0;
uint32_t cont_passo_step = 0;

float dac_speed = 0;
uint8_t dac_change = 0;

float angle_body[SIZE_ANGLE];

int flag_teste = 2;

void print_variables (void){
	switch (flag_teste){
		case 1:
			snprintf(buf, sizeof(buf), "\r\nTESTE DE RAMPA:\r\nRampa: %.3f segundos\r\nManter constante: %.3f segundos\r\nVelocidade Pico: %.3f RPM\r\n", ramp, constant, speed_final);
			printf(buf);
		break;
		case 2:
			snprintf(buf, sizeof(buf), "\r\nTESTE DE STEP VARIAVEL:\r\nTempo de CADA Step: %.3f segundos\r\nIteracoes do Step: %u vezes\r\nVelocidade Pico: %.3f RPM\r\nValor do Passo: %.3f", constant, divisions, speed_final, passo_step);
			printf(buf);
		break;
		case 3:
			snprintf(buf, sizeof(buf), "\r\nTESTE DE STEP CONSTANTE:\r\nTempo de Step: %.3f segundos\r\nVelocidade Pico: %.3f RPM\r\n", constant, speed_final);
			printf(buf);
		break;
		case 4:
			snprintf(buf, sizeof(buf), "\r\nTESTE DE DAC PELA SERIAL:\r\nAguardando comando go\r\n");
			printf(buf);
		break;
	}
}

void rampa (void){
	//Primeira Rampa, ascenção:
	if (ramp_timer > cont_timer){
		speed = update_accel_motor(acceleration,speed, t_seconds);
	}
	//Manter constante:
	else if ( (ramp_timer == cont_timer) && ((ramp_timer + constant_timer) > cont_timer) ){
		update_speed_motor(speed);
		gpio_set_pin_low(LED0_GPIO);
	}
	//Segunda Rampa, descensão:
	else if ( ((ramp_timer + constant_timer) <= cont_timer) && (ramp_timer_second > cont_timer) ){
		gpio_set_pin_high(LED0_GPIO);
		speed = update_accel_motor((-acceleration),speed, t_seconds);
	}
	//Segundo manter constante:
	else if ( (ramp_timer_second == cont_timer) && ( constant_timer_second > cont_timer ) ){
		gpio_set_pin_low(LED0_GPIO);
		update_speed_motor(speed);
	}
	// Terceira Rampa, ascenção final:
	else if ( ( constant_timer_second <= cont_timer ) && ( final_time > cont_timer ) ){
		gpio_set_pin_high(LED0_GPIO);
		speed = update_accel_motor(acceleration,speed, t_seconds);
	}
	// Retornando ao estado de repouso:
	else if ( (final_time >= cont_timer) && (constant_timer_second <= cont_timer) ){
		speed = 0;
		update_speed_motor(speed);
		flag_go = 0;
		acceleration = 0;
		printf("STOP\r\n");
		gpio_set_pin_high(LED1_GPIO);
	}
}

void step_variavel(void){
	if (cont_timer == (step_timer*cont_step) ){
		if (cont_step == 0){
			speed = speed_final;
		} else if ( cont_step % 2 == 0 ){
			//speed = speed_final/par_divisor;
			//speed = speed_final - ( (cont_step-1) *passo_step);
			speed = speed_final - ( (float) ( (float)((cont_step-1)*0.5) + 0.5) * passo_step);
			par_divisor++;
			gpio_toggle_pin(LED0_GPIO);
		} else {
			//speed = speed_final*impar_mult;
			speed = speed_final + ( (float) ( (float)(cont_step*0.5) + 0.5) * passo_step);
			impar_mult++;
			gpio_toggle_pin(LED0_GPIO);
		}
		
		if ( (2*divisions + 1) <= cont_step){
			flag_go = 0;
			speed = 0;
			printf("STOP\r\n");
			gpio_set_pin_high(LED1_GPIO);
			gpio_set_pin_high(LED0_GPIO);
		}
		
		update_speed_motor(speed);
		cont_step++;
	}
}

void step_simple(void){
	if ( cont_timer == 0 ){
		speed = speed_final;
		update_speed_motor(speed);
	} else if ( cont_timer == step_timer ){
		flag_go = 0;
		speed = 0;
		update_speed_motor(speed);
		printf("STOP\r\n");
		gpio_set_pin_high(LED1_GPIO);
	}
}

void receive_dac(void){
	if (dac_change == 1){
		update_speed_motor_dac(dac_speed);
		dac_change = 0;
		
		measure_speed_duty(&d);
		measure_speed_freq(&f);
		angleXY_dynamic(&angle_body, sizeof(angle_body));
		//snprintf(buf, sizeof(buf), "Duty = %.3f\tFrequency = %.3f\r\n", d[0], f);
		snprintf(buf, sizeof(buf), "%d %.3f %.3f %.3f\r\n", dac_val,f, d[0], angle_body[0]);
		printf(buf);
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();

	/* Insert application code here, after the board has been initialized. */
	
	configure_console();
	config_timer();
	config_motor();
	//config_IMU();
	
	printf("Motor Model Identification - Com Utilizando Rampa\r\n");
	print_variables();
	
	gpio_set_pin_low(LED1_GPIO);
	gpio_set_pin_low(LED0_GPIO);
	gpio_set_pin_low(LED2_GPIO);
	
	while(1){
		if ( flag_time_sample ){			
			if (flag_go){
				switch (flag_teste){
					case 1:
						rampa();
					break;
					case 2:
						step_variavel();
						if ( (flag_go == 0) && ( speed_final > 0 ) ){
							speed_final = -speed_final;
							flag_go = 1;
						}
					break;
					case 3:
						step_simple();
					break;
				}
				cont_timer++;
			}
			
			if ( (flag_go) & (flag_teste != 4) ){
				measure_speed_duty(&d);
				measure_speed_freq(&f);
				//snprintf(buf, sizeof(buf), "Duty = %.3f\tFrequency = %.3f\r\n", d[0], f);
				snprintf(buf, sizeof(buf), "%d %.3f %.3f\r\n", dac_val,f, d[0]);
				printf(buf);
			}
			
			flag_time_sample = 0;
		}
		
		if ( (flag_go) & (flag_teste == 4) ){
			receive_dac();
		}
		
		if (uc_flag){
			uc_flag = 0;
			switch (keys[0]){
				//EXIBIR VALORES:
				case 'v':
					print_variables();
				break;
				//VELOCIDADE:
				case 's':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '-') && (keys[0] <= '9') ){
						speed_final = atoff(keys);
						//print_variables();
					}
				break;
				//TEMPO DE RAMPA:
				case 'r':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '0') && (keys[0] <= '9') ){
						ramp = atoff(keys);
						//print_variables();
					}
				break;
				//TEMPO CONSTANTE:
				case 'c':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '0') && (keys[0] <= '9') ){
						constant = atoff(keys);
						//print_variables();
					}
				break;
				//TIPO DE TESTE:
				case 't':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '1') && (keys[0] <= '4') ){
						flag_teste = atoi(keys);
						//print_variables();
					}
				break;
				//ITERAÇÕES NO STEP:
				case 'd':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '0') && (keys[0] <= '9') ){
						divisions = atoi(keys);
						//print_variables();
					}
				break;
				//PASSO DO STEP:
				case 'p':
					shift_left(&keys, sizeof(keys));
					if ( (keys[0] >= '-') && (keys[0] <= '9') ){
						passo_step = atoff(keys);
						//print_variables();
					}
				break;
				//RECEBENDO DAC VIA SERIAL:
				case 'D':
					if ( (flag_teste == 4) && (flag_go) ){	//Necessario estar em teste de DAC e GO ativado
						shift_left(&keys, sizeof(keys));
						if ( (keys[0] >= '-') && (keys[0] <= '9') ){
							dac_speed = atoff(keys);
							dac_change = 1;
						}
					}
				break;
				//RECEBENDO STOP PARA PARAR O TESTE: (Utilizado no teste 4)
				case 'S':
					if ( (keys[1] == 'T') && (keys[2] == 'O') && (keys[3] == 'P') ){
						if (flag_teste == 4){
							flag_go = 0;
							gpio_set_pin_high(LED1_GPIO);
						}
					}
				break;
				//INICIANDO O TESTE:
				case 'g':
					if (keys[1] == 'o'){						
						if (flag_teste == 1){
							//Definindo Constantes de Tempo:
							ramp_timer = (ramp*TIMER_CONSTANT)/TIME_SAMPLE;
							constant_timer = (constant*TIMER_CONSTANT)/TIME_SAMPLE;
							ramp_timer_second = 3*ramp_timer + constant_timer;
							constant_timer_second = ramp_timer_second + constant_timer;
							final_time = constant_timer_second + ramp_timer;
							//Rampa de aceleração:
							acceleration = (speed_final) / ramp;
						} else if (flag_teste == 2){
							//Calculando tempo de cada step:
							step_timer = (constant*TIMER_CONSTANT)/TIME_SAMPLE;
							cont_step = 0;
							par_divisor = 2;
							impar_mult = 2;
						} else if (flag_teste == 3){
							//Calculando tempo do cada step:
							step_timer = (constant*TIMER_CONSTANT)/TIME_SAMPLE;
						} else if ( flag_teste == 4 ){
							speed = 0;
							init_angle_dynamic(&angle_body, sizeof(angle_body));	//Esperar convergencia do angulo
							printf("GO\r\n");
						}
						
						flag_go = 1;
						flag_time_sample = 0;
						cont_timer = 0;
						speed = 0;
						update_speed_motor(speed);
						gpio_set_pin_low(LED1_GPIO);
					}
				break;
			}
		}
	}
}
