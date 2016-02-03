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
#include <asf.h>

void pin_riseedge_handler(uint32_t id, uint32_t mask);

void pin_riseedge_handler(uint32_t id, uint32_t mask){
	if ( (ID_PIOA == id) && (PIO_PA16 == mask) )
	{
		gpio_toggle_pin(LED0_GPIO);
		gpio_toggle_pin(LED1_GPIO);
	}
	if ( (ID_PIOA == id) && (PIO_PA17 == mask) )
	{
		gpio_toggle_pin(LED0_GPIO);
		gpio_toggle_pin(LED1_GPIO);
	}
}

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).

	sysclk_init();
	board_init();

	// Insert application code here, after the board has been initialized.
	
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_input(PIOA, PIO_PA16, PIO_DEFAULT);
	pio_pull_down(PIOA, (PIO_PA16), ENABLE);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA16, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA,PIO_PA16);
	
	pio_set_input(PIOA, PIO_PA17, PIO_DEFAULT);
	pio_pull_down(PIOA, (PIO_PA17), ENABLE);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA17, PIO_IT_RISE_EDGE, pin_riseedge_handler);
	pio_enable_interrupt(PIOA,PIO_PA17);
	
	NVIC_EnableIRQ(PIOA_IRQn);
	
	gpio_set_pin_low(LED0_GPIO);
	gpio_set_pin_high(LED1_GPIO);
	
	while(1){
		
	}
}
