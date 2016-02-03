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
#define PI						3.141592654

void config_lcd(void);
static void configure_console(void);

static float
acel_adxl[3],
gyro_itg[3];

float angleYX = 0;

uint32_t cont_ticks = 0;

struct ili9225_opt_t g_ili9225_display_opt;

/** Global timestamp in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;
/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void){
	g_ul_ms_ticks++;
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
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(UART0, &uart_serial_options);
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
			gpio_set_pin_low(LED2_GPIO);
		}
	}

	/* Insert application code here, after the board has been initialized. */
	
	configure_console();
	config_lcd();
	
	twi_init();
	/*TWI0 / TWI1:
	- Clock:	PA4 / PB5 (Laranja)
	- Data:		PA3 / PB4 (Cinza)
	- Terra:	GND (Preta)
	- VCC:		3V3 (Branco)*/
	adxl_init();
	itg_init();
	
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10,10, (uint8_t *)"Teste TWI IMU");
	
	char buf[100];
	uint32_t cont = 0;
	cont_ticks = g_ul_ms_ticks;
	
	while(1){
		
		if ( (g_ul_ms_ticks - cont_ticks) >= 20 ){
			cont_ticks = g_ul_ms_ticks;
			get_all_acel_value(&acel_adxl);
			acel_adxl[0] = acel_adxl[0];// - 400.0;
			acel_adxl[1] = acel_adxl[1];// + 500.0;
			acel_adxl[2] = acel_adxl[2];// - 5000.0;
			angleYX = atanf(acel_adxl[0]/acel_adxl[1]) * (180.0/PI);
			get_all_gyro_value(&gyro_itg);
			snprintf(buf, sizeof(buf),"%.4f\n", angleYX);
			printf(buf);
			cont++;
		}
		
		if (cont >= 50){
			cont = 0;
			/*snprintf(buf, sizeof(buf),"Acel(mG):\nx=%.1f\ny=%.1f\nz=%.1f\n\nGyro(Graus/s):\nx=%.1f\ny=%.1f\nz=%.1f\n\nTicks=%1u ms"
			, acel_adxl[0], acel_adxl[1], acel_adxl[2], gyro_itg[0], gyro_itg[1], gyro_itg[2], (g_ul_ms_ticks - cont_ticks));*/
			snprintf(buf, sizeof(buf),"Acel(mG):\nx=%.1f\ny=%.1f\nz=%.1f\nYX = %.3f\n\nGyro(Graus/s):\nx=%.1f\ny=%.1f\nz=%.1f"
			, acel_adxl[0], acel_adxl[1], acel_adxl[2], angleYX, gyro_itg[0], gyro_itg[1], gyro_itg[2]);
			ili9225_set_foreground_color(COLOR_WHITE);
			ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
			ili9225_set_foreground_color(COLOR_BLACK);
			ili9225_draw_string(10, 30, (uint8_t*) buf);
		}
		
	}
}