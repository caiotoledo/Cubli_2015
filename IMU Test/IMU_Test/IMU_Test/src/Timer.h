/*
 * Timer.h
 *
 * Created: 14/09/2015 20:40:51
 *  Author: Caio
 */ 


#ifndef TIMER_H_
#define TIMER_H_

//#define TIME_SAMPLE 200		//20.0 ms
#define TIME_SAMPLE 100		//10.0 ms
//#define TIME_SAMPLE 5000	//500.0 ms

#define TIMER_CONSTANT		10000 //CONFIGURADO PARA 0,1 ms

void mdelay(uint32_t ul_dly_ticks);
void config_timer(void);
uint8_t get_flag_sample(void);
void set_flag_sample(uint8_t flag);

extern volatile uint8_t flag_time_sample;
extern volatile uint32_t g_ul_ms_ticks;

#endif /* TIMER_H_ */