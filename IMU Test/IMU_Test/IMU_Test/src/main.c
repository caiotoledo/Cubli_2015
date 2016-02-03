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
#include "UART_Comm.h"
#include "IMU.h"
#include "Timer.h"

void verificar_twi(void);
void config_interrupt (void);
void pin_riseedge_handler(uint32_t id, uint32_t mask);

#define ENCODER_INC		1024
#define VOLTACOMPLETA	360

char buf[200];

float angle = 0;
uint8_t flag_go = 0;

float acel_adxl_high[3];
float gyro_itg_high[3];
float angle_nofilter_high = 0;
float angle_filter_high = 0;
float gyroz_high = 0;

float acel_adxl_low[3];
float gyro_itg_low[3];
float angle_nofilter_low = 0;
float angle_filter_low = 0;
float gyroz_low = 0;

float angle_dynamic[SIZE_ANGLE];

uint8_t twi_status = 1;
float time_test = 1.0;
uint32_t cont_test = 0;
uint32_t cont_timer = 0;

const static float passo_encoder = ((float) VOLTACOMPLETA/ENCODER_INC);
static float degrees_cont = 0;
static float ant_degrees_cont = 0;

uint32_t flag_teste = 0;

uint32_t cont_ticks = 0;
uint32_t cont_t = 0;

void verificar_twi(void){
	if (twi_status != TWI_SUCCESS){
		gpio_set_pin_low(LED2_GPIO);
		printf("STOP\r\n");
	}
}

void pin_riseedge_handler(uint32_t id, uint32_t mask){
	if ( (id == ID_PIOA) && ( mask == PIO_PA17 ) ){
		degrees_cont -= passo_encoder;
		gpio_toggle_pin(LED1_GPIO);
	} else if ( (id == ID_PIOA) && ( mask == PIO_PA18 ) ){
		degrees_cont += passo_encoder;
		gpio_toggle_pin(LED2_GPIO);
	}
}

void config_interrupt (void){
	pmc_enable_periph_clk(ID_PIOA);
	
	pio_set_input(PIOA, PIO_PA17, PIO_DEFAULT);
	pio_pull_down(PIOA, PIO_PA17, ENABLE);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA17, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA, PIO_PA17);
	
	pio_set_input(PIOA, PIO_PA18, PIO_DEFAULT);
	pio_pull_down(PIOA, PIO_PA18, ENABLE);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA18, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA, PIO_PA18);
	
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 1);
	NVIC_EnableIRQ(PIOA_IRQn);
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();

	/* Insert application code here, after the board has been initialized. */
	
	config_timer();
	configure_console();
	config_interrupt();
	
	/*TWI0:
	- Clock:	PA4 (Laranja)
	- Data:		PA3 (Cinza)
	- Terra:	GND (Preta)
	- VCC:		3V3 (Branco)*/
	twi_status = twi_init();
	if (twi_status == TWI_SUCCESS)
	{
		adxl_init();
		angleXY(&angle_filter_high,ADXL_ADDR_HIGH);	//Angle initialization
		angleXY(&angle_filter_low,ADXL_ADDR_LOW);	//Angle initialization
		itg_init();
	} else {
		printf("Erro TWI");
		while(1){
			gpio_set_pin_low(LED2_GPIO);
		}
	}
	
	printf("OK!\r\n");
	cont_t = g_ul_ms_ticks;
	
	while(1){
		
		if ( (g_ul_ms_ticks - cont_t) >= 5000)
		{
			gpio_toggle_pin(LED0_GPIO);
			cont_t = g_ul_ms_ticks;
		}

		if (flag_go){
			if (flag_time_sample){
				flag_time_sample = 0;
				cont_ticks = g_ul_ms_ticks;
				get_all_acel_value(&acel_adxl_high, ADXL_ADDR_HIGH);
				get_all_acel_value(&acel_adxl_low, ADXL_ADDR_LOW);
				
				/*twi_status |= angleXY_filter(&angle_filter_high, &gyroz_high, ADXL_ADDR_HIGH, ITG_ADDR_HIGH);
				twi_status |= angleXY_filter(&angle_filter_low, &gyroz_low, ADXL_ADDR_LOW, ITG_ADDR_LOW);
				
				twi_status |= angleXY(&angle_nofilter_high,ADXL_ADDR_HIGH);
				twi_status |= angleXY(&angle_nofilter_low,ADXL_ADDR_LOW);*/
				
				twi_status |= angleXY_dynamic(&angle_dynamic);
				verificar_twi();
				
				//Enviando via Serial os Valores:
				snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.1f %.1f\r\n",acel_adxl_high[0], acel_adxl_high[1],
																										acel_adxl_low[0], acel_adxl_low[1],
																										gyroz_high, gyroz_low,
																										angle_filter_high, angle_filter_low,
																										angle_nofilter_high, angle_nofilter_low,
																										angle_dynamic[0],
																										degrees_cont,
																										( (degrees_cont - ant_degrees_cont)/0.02)  );
				printf(buf);
				cont_timer++;
				ant_degrees_cont = degrees_cont;
				cont_ticks = g_ul_ms_ticks - cont_ticks;
				if (cont_timer >= cont_test){
					flag_go = 0;
					printf("STOP\r\n");
					gpio_set_pin_high(LED0_GPIO);
				}
			}
		}
		
		//TRATANDO VALORES RECEBIDOS NA SERIAL:
		if (uc_flag){
			uc_flag = 0;
			
			switch (keys[0]){
				case 'g':
					if ( keys[1] == 'o'){
						flag_go = 1;
						flag_time_sample = 0;
						cont_test = (time_test*TIMER_CONSTANT)/TIME_SAMPLE;
						cont_timer = 0;
						gpio_set_pin_low(LED0_GPIO);
						degrees_cont = 0;
						ant_degrees_cont = 0;
						angleXY(&angle_filter_high,ADXL_ADDR_HIGH);	//Angle initialization
						angleXY(&angle_filter_low,ADXL_ADDR_LOW);	//Angle initialization
						
						float angle_init_high, angle_init_low;
						angleXY(&angle_init_high,ADXL_ADDR_HIGH);
						angleXY(&angle_init_low,ADXL_ADDR_HIGH);
						angle_dynamic[0] = (angle_init_high + angle_init_low)/2;
						//angle_dynamic[0] = 0;
					}
				break;
				
				case 'S':
					if ( (keys[1] == 'T') && (keys[2] == 'O') && (keys[3] == 'P') ){
						flag_go = 0;
						gpio_set_pin_high(LED0_GPIO);
					}
				break;
				
				case 't':
					shift_left(keys, sizeof(keys));
					if ( (keys[0] >= '0') && (keys[0] <= '9') ){
						time_test = atoff(keys);
					}
				break;
			}
		}
		
	}
}
