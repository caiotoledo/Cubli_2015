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
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "IMU.h"

#define CONF_UART_BAUDRATE		115200

void config_lcd(void);
static void configure_console(void);
void config_interrupt(void);
void pin_riseedge_handler(uint32_t id, uint32_t mask);

float
g_x, g_y, g_z;

static float 
acel_adxl[3],
gyro_itg[3];

uint8_t flag = 0;

uint8_t flag_adxl = 0;
uint8_t flag_itg = 0;
uint8_t primeiro = 1;

uint32_t cont_gyro = 0;
uint32_t cont_acel = 0;
uint32_t cont_ticks = 0;
uint32_t timer_itg = 0;
uint32_t timer_adxl = 0;

struct ili9225_opt_t g_ili9225_display_opt;

/** Global timestamp in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;
/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

/**
 *  \brief Wait for the given number of milliseconds (using the dwTimeStamp
 *         generated by the SAM microcontrollers' system tick).
 *  \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

void pin_riseedge_handler(uint32_t id, uint32_t mask){
	if ( ( ID_PIOA == id ) && (INT_ADXL == mask) ){
		flag_adxl = 1;
		timer_adxl = g_ul_ms_ticks - cont_ticks;
	} else if ( ( ID_PIOA == id ) && (INT_ITG == mask) ){
		flag_itg = 1;
		timer_itg = g_ul_ms_ticks - cont_ticks;
	}
}

/**
 * \brief Override SPI handler.
 */
void SPI_Handler(void)
{
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
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(UART0, &uart_serial_options);
}

void config_interrupt(void)
{
	pmc_enable_periph_clk(ID_PIOA);
	
	//Configurando Interrupção do ADXL:
	pio_set_input(PIOA, INT_ADXL, PIO_DEFAULT);
	pio_pull_down(PIOA, (INT_ADXL), ENABLE);
	pio_handler_set(PIOA, ID_PIOA, INT_ADXL, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	//pio_handler_set(PIOA, ID_PIOA, INT_ADXL, PIO_IT_HIGH_LEVEL, pin_riseedge_handler);
	pio_enable_interrupt(PIOA,INT_ADXL);
	//Configurando Interrupção do ITG:
	pio_set_input(PIOA, INT_ITG, PIO_DEFAULT);
	//pio_pull_down(PIOA, (INT_ITG), ENABLE);
	pio_handler_set(PIOA, ID_PIOA, INT_ITG, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	//pio_handler_set(PIOA, ID_PIOA, INT_ITG, PIO_IT_HIGH_LEVEL, pin_riseedge_handler);
	pio_enable_interrupt(PIOA,INT_ITG);
	
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
}

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).

	sysclk_init();
	board_init();

	// Insert application code here, after the board has been initialized.
	
	configure_console();
	//config_lcd();			//PRIORIDADE DE INTERRUPÇÃO MODIFICADA DE 0 PARA 1 !!!
	twi_init();
	/*TWI0 / TWI1:
	- Clock:	PA4 / PB5 (Laranja)
	- Data:		PA3 / PB4 (Cinza)
	- Terra:	GND (Preta)
	- VCC:		3V3 (Branco)
	- INT_ITG:	PA18 (Vermelhor)
	- INT_ADXL:	PA17 (Amarelo)*/
	adxl_init();
	itg_init();
	
	/*ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10,10, (uint8_t *)"Teste TWI IMU");*/
	
	printf("Start!");
	
	char buffer[100];
	
	config_interrupt();
	clear_interrupt();

	while(1){
		if (flag_adxl){
			get_all_acel_value(&acel_adxl);
			acel_adxl[2] = acel_adxl[2] - 3290.0;
			flag_adxl = 0;
			flag |= 0x80;
			cont_acel++;
		}
		if (flag_itg){
			get_all_gyro_value(&gyro_itg);
			flag_itg = 0;
			flag |= 0x01;
			cont_gyro++;
		}
		if ( flag == 0x81)
		{
			flag = 0;
			if (cont_acel != cont_gyro)
			{
				if (primeiro) {
					primeiro = 0;
				} else {
					gpio_toggle_pin(LED0_GPIO);	
				}
			}
			cont_acel = 0;
			cont_gyro = 0;
			
			
			snprintf(buffer, sizeof(buffer),"Acel(mG):\nx=%.1f\ny=%.1f\nz=%.1f\n\nGyro(°/s):\nx=%.1f\ny=%.1f\nz=%.1f\n\nTicks=%1u ms"
			, acel_adxl[0], acel_adxl[1], acel_adxl[2], gyro_itg[0], gyro_itg[1], gyro_itg[2], (g_ul_ms_ticks - cont_ticks));
			cont_ticks = g_ul_ms_ticks;
			/*ili9225_set_foreground_color(COLOR_WHITE);
			ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
			ili9225_set_foreground_color(COLOR_BLACK);
			ili9225_draw_string(10, 30, (uint8_t*) buffer);*/
			printf(buffer);
		} else {
			if ( (g_ul_ms_ticks - cont_ticks) > 40 )
			{
				clear_interrupt();
				cont_ticks = g_ul_ms_ticks;
				gpio_toggle_pin(LED1_GPIO);
			}
		}
	}
}
