/*
 * UART_Comm.h
 *
 * Created: 19/09/2015 16:25:37
 *  Author: Caio
 */ 


#ifndef UART_COMM_H_
#define UART_COMM_H_

#define CONF_UART_BAUDRATE		115200//9600
#define CHAR_LENGHT				8
#define STOPBITS				1
#define PARITY					UART_MR_PAR_NO

void configure_console(void);
void clear_keys(void);
void shift_left (uint8_t *items, uint8_t size);

extern volatile uint8_t uc_flag;
extern uint8_t keys[100];

#endif /* UART_COMM_H_ */