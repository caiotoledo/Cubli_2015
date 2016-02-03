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
#include "IMU.h"

#define CONF_UART_BAUDRATE		9600
#define ADXL_TEST				ADXL_ADDR_HIGH
#define ITG_TEST				ITG_ADDR_HIGH

void config_lcd(void);
static void configure_console(void);

static float
acel_adxl[3],
gyro_itg[3];

float angle = 0;
float gyroz = 0;
float angle_nofilter = 0;

uint32_t cont_time = 0;

char buf[100];
uint32_t status_transfer = 0;

uint32_t cont_ticks = 0;

struct ili9225_opt_t g_ili9225_display_opt;

/** Global timestamp in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;
uint8_t flag_tick = 0;
/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void){
	g_ul_ms_ticks++;
	
	if ( (g_ul_ms_ticks - cont_ticks) >= TIMER_SAMPLE)
	{
		flag_tick = 1;
		cont_ticks = g_ul_ms_ticks;
	}
}

/**
 *  \brief Wait for the given number of milliseconds (using the dwTimeStamp
 *         generated by the SAM microcontrollers' system tick).
 *  \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t ul_dly_ticks){
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

/**
 * \brief Override SPI handler.
 */
void SPI_Handler(void){
	ili9225_spi_handler();
}

void config_lcd(void){
	
	/* Initialize display parameter */
	g_ili9225_display_opt.ul_width = ILI9225_LCD_WIDTH;
	g_ili9225_display_opt.ul_height = ILI9225_LCD_HEIGHT;
	g_ili9225_display_opt.foreground_color = COLOR_BLACK;
	g_ili9225_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9225_init(&g_ili9225_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	/* Turn on LCD */
	ili9225_display_on();
	
	/* Draw filled rectangle with white color */
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 0, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = 8,
		.stopbits = 1,
		.paritytype = UART_MR_PAR_NO,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(UART0, &uart_serial_options);
}

void button(void){
	if (gpio_pin_is_low(GPIO_PUSH_BUTTON_2)){
		mdelay(50);
		if (!gpio_pin_is_low(GPIO_PUSH_BUTTON_2)){
			return;
		}
		while(gpio_pin_is_low(GPIO_PUSH_BUTTON_2)){
			mdelay(50);
		}
		
		if (status_transfer == 0){
			//Start Matlab Receiver:
			/*snprintf(buf, sizeof(buf), "START\r\n");
			printf(buf);*/
			status_transfer = 1;
			gpio_set_pin_low(LED1_GPIO);
			cont_time = 0;
		}
		else{
			//Stop Matlab Receiver:
			snprintf(buf, sizeof(buf), "STOP\r\n");
			printf(buf);
			status_transfer = 0;
			gpio_set_pin_high(LED1_GPIO);
		}
		
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();

	/* Insert application code here, after the board has been initialized. */
	
	configure_console();
	config_lcd();
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10,10, (uint8_t *)"Filtro Complementar");
	
	/*TWI0:
	- Clock:	PA4 (Laranja)
	- Data:		PA3 (Cinza)
	- Terra:	GND (Preta)
	- VCC:		3V3 (Branco)*/
	uint8_t twi_status = twi_init();
	if (twi_status == TWI_SUCCESS)
	{
		adxl_init();
		angleXY(&angle,ADXL_TEST);	//Angle initialization		
		itg_init();
	} else {
		printf("Erro TWI");
		while(1){
			gpio_set_pin_low(LED2_GPIO);
		}
	}
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
			gpio_set_pin_low(LED2_GPIO);
		}
	}
	
	uint32_t cont = 0;
	cont_ticks = g_ul_ms_ticks;
	
	while(1){
		
		if ( flag_tick ){
			flag_tick = 0;
			
			get_all_acel_value(&acel_adxl, ADXL_TEST);
			/*acel_adxl[0] = acel_adxl[0] + ADXL_OFSTX_HIGH;
			acel_adxl[1] = acel_adxl[1] + ADXL_OFSTY_HIGH;
			acel_adxl[2] = acel_adxl[2] ;*/
			
			uint8_t test = angleXY_filter(&angle, &gyroz, ADXL_TEST, ITG_TEST);
			if (test != TWI_SUCCESS){
				gpio_set_pin_low(LED2_GPIO);
			}
			
			test = angleXY(&angle_nofilter,ADXL_TEST);
			
			get_all_gyro_value(&gyro_itg, ITG_TEST);
			if (status_transfer)
			{
				//snprintf(buf, sizeof(buf),"%.3f %.3f\r\n",angle,gyroz);
				//snprintf(buf, sizeof(buf),"%.3f %.3f %.3f %.3f\r\n",angle,gyroz,acel_adxl[0],acel_adxl[1]);
				snprintf(buf, sizeof(buf),"%.3f %.3f %.3f %.3f %.3f\r\n",angle,gyroz,acel_adxl[0],acel_adxl[1], angle_nofilter);
				printf(buf);
				cont_time++;
			}
			cont++;
		}
		
		if (cont*TIMER_SAMPLE >= 1000){
			gpio_toggle_pin(LED0_GPIO);
			cont = 0;
			snprintf(buf, sizeof(buf),"Acel(mG):\nx=%.1f\ny=%.1f\nz=%.1f\nXY = %.3f\n\nGyro(Graus/s):\nx=%.1f\ny=%.1f\nz=%.1f\nTime = %u ms"
			, acel_adxl[0], acel_adxl[1], acel_adxl[2], angle_nofilter, gyro_itg[0], gyro_itg[1], gyroz, (cont_time*20) );
			/*snprintf(buf, sizeof(buf),"Acel(mG):\nx=%.1f\ny=%.1f\nz=%.1f\n\nGyro(�/s):\nx=%.1f\ny=%.1f\nz=%.1f\n\nTicks=%1u ms"
			, acel_adxl[0], acel_adxl[1], acel_adxl[2], gyro_itg[0], gyro_itg[1], gyro_itg[2], (g_ul_ms_ticks - cont_ticks));*/
			ili9225_set_foreground_color(COLOR_WHITE);
			ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
			ili9225_set_foreground_color(COLOR_BLACK);
			ili9225_draw_string(10, 30, (uint8_t*) buf);
		}
		
		button();
		
	}
}