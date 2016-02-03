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
#include "IMU.h"
#include "Motor.h"
#include "UART_Comm.h"
#include "Timer.h"

void pisca_LED (void);
void init_control (void);
void receive_serial (void);
void print_constants (void);

#define Angle_low	(-0.00349066)
#define Angle_High	(0.00349066)

//CONSTANTES DO PID:
#define SATURACAO_PID		500	
float Kd = -13.0677;
float Kp = -20;
float Ki = -50;
float Kt = 0.5;
float Control_I = 0;
float Control_P = 0;
float Control_D = 0;
float setpoint = 0.0261799;

uint8_t flag_go = 0;	//Flag para iniciar controle

//VARIAVEIS DE ESTADO:
float angle_body[SIZE_ANGLE];
float speed_motor[SIZE_SPEED];
float output = 0;
float output_vetor[3];
float gyroZ = 0;
	
float time_test = 5;
uint32_t cont_test = 0;
uint32_t cont_timer = 0;

char buf[250];
//char buffer[50];
uint32_t cont_t = 0;
uint32_t cont_t_exib = 0;
uint32_t cont_LED = 0;

void init_control (void){
	gpio_set_pin_low(LED0_GPIO);
	
	init_speed_motor(&speed_motor);		//Zerar speed_motor
	output = 0;							//Zerar Output
	output_vetor[0] = 0;
	output_vetor[1] = 0;
	output_vetor[2] = 0;
	init_angle_dynamic(&angle_body);	//Esperar Convergencia do Angulo
	Control_I = 0;
	Control_P = 0;
	Control_D = 0;
	
	gpio_set_pin_high(LED0_GPIO);
}

void print_constants (void){
	snprintf(buf, sizeof(buf), "COMPENSADOR:\r\nTempo de Teste: %.3f\r\nKd = %.3f\r\nKi = %.3f\r\nKp = %.3f\r\nKt = %.3f\r\nSP = %.3f\r\n",
								time_test, Kd, Ki, Kp, Kt, setpoint);
	printf(buf);
}

void receive_serial (void){
	if (uc_flag){
		uc_flag = 0;
		
		switch (keys[0]){
			case 'g':
				if ( keys[1] == 'o' ){					
					init_control();
					
					flag_go = 1;
					cont_test = (time_test*TIMER_CONSTANT)/TIME_SAMPLE;
					cont_timer = 0;
				
					gpio_set_pin_low(LED1_GPIO);		
				}
			break;
			
			case 'S':
				if ( (keys[1] == 'T') && (keys[2] == 'O') && (keys[3] == 'P') ){
					flag_go = 0;
					gpio_set_pin_high(LED1_GPIO);
				}
			break;
			
			case 't':
				shift_left(keys, sizeof(keys));
				if ( (keys[0] >= '0') && (keys[0] <= '9') ){
					time_test = atoff(keys);
				}
			break;
			
			case 'c':
				print_constants();
			break;
			
			case 's':
				shift_left(keys, sizeof(keys));
				if (keys[0] == 'p'){
					shift_left(keys, sizeof(keys));
					if ( (keys[0] >= '-') && (keys[0] <= '9') ){
						setpoint = atoff(keys);
					}
				}
			break;
			
			case 'K':
				shift_left(keys, sizeof(keys));
				switch (keys[0]){
					
					case 'd':
						shift_left(keys, sizeof(keys));
						if ( (keys[0] >= '-') && (keys[0] <= '9') ){
							Kd = atoff(keys);
						}
					break;
					
					case 'i':
						shift_left(keys, sizeof(keys));
						if ( (keys[0] >= '-') && (keys[0] <= '9') ){
							Ki = atoff(keys);
						}
					break;
					
					case 't':
						shift_left(keys, sizeof(keys));
						if ( (keys[0] >= '-') && (keys[0] <= '9') ){
							Kt = atoff(keys);
						}
					break;
					
					case 'p':
						shift_left(keys, sizeof(keys));
						if ( (keys[0] >= '-') && (keys[0] <= '9') ){
							Kp = atoff(keys);
						}
					break;
					
				}
			break;
		}
	}
}

void controle(void){
	if (flag_time_sample){
		float v = 0;
		cont_t = g_ul_ms_ticks;
		angleXY_dynamic(&angle_body);
		measure_speed(&speed_motor);
						 
		shift_right(output_vetor,sizeof(output_vetor));
		
		gyroZ = get_gyro_value('Z',ITG_ADDR_LOW);
		Control_P = Kp*(setpoint-angle_body[0]);
		Control_D = Kd*gyroZ;
		
		output_vetor[0] = Control_D + Control_I + Control_P;
		v = output_vetor[0];
		if ( v > SATURACAO_PID ){
			output_vetor[0] = SATURACAO_PID;
		} 
		else if ( v < (-SATURACAO_PID) ){
			output_vetor[0] = -SATURACAO_PID;
		}
		update_speed_motor_dac(output_vetor[0]);
		
		Control_I = Control_I + Ki*(setpoint-angle_body[0]) + Kt*( output_vetor[0] - v );
						
		snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %u\r\n", angle_body[0], speed_motor[0], gyroZ, output_vetor[0], cont_t_exib);
		printf(buf);
		
		cont_t_exib = g_ul_ms_ticks - cont_t;
		
		if ( (angle_body[0] > (setpoint+Angle_low) ) && (angle_body[0] < (setpoint+Angle_High) ) ){
			gpio_set_pin_low(LED0_GPIO);
		} 
		else{
			gpio_set_pin_high(LED0_GPIO);
		}
		
		flag_time_sample = 0;
		cont_timer++;	
	}
}

void pisca_LED (void){
	if ( (g_ul_ms_ticks - cont_LED) >= 5000 ){
		gpio_toggle_pin(LED2_GPIO);
		cont_LED = g_ul_ms_ticks;
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();

	/* Insert application code here, after the board has been initialized. */
		
	gpio_set_pin_low(LED2_GPIO);
	config_timer();
	mdelay(5000);
	configure_console();
	config_motor();
	config_IMU();
	
	print_constants();
		
	//cont_LED = g_ul_ms_ticks;
	while(1){
		pisca_LED();
		
		receive_serial();
		
		if (flag_go){
			controle();
			
			if (cont_timer >= cont_test){
				flag_go = 0;
				printf("STOP\r\n");
				gpio_set_pin_high(LED1_GPIO);
				gpio_set_pin_high(LED0_GPIO);
				update_speed_motor(0);
			}
		}
	}
}