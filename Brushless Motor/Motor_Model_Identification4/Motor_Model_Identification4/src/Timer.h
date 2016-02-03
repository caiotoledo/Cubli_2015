/*
 * Timer.h
 *
 * Created: 14/09/2015 20:40:51
 *  Author: Caio
 */ 


#ifndef TIMER_H_
#define TIMER_H_

//#define TIME_SAMPLE 200		//20.0 ms
//#define TIME_SAMPLE 150		//15.0 ms
#define TIME_SAMPLE 100		//10.0 ms
//#define TIME_SAMPLE 5000	//500.0 ms

#define TIMER_CONSTANT		10000 //CONFIGURADO PARA 0,1 ms

#define PRIORITY_TICK		0
#define PRIORITY_MEASURE	1
#define PRIORITY_COMM		2

void mdelay(uint32_t ul_dly_ticks);
void config_timer(void);
uint8_t get_flag_sample(void);
void set_flag_sample(uint8_t flag);
void shift_right (float *vetor, uint32_t size );
void filter_lowpass(float *vetor, float *H, uint32_t size);

//Constantes para Filtros:
static const float Hd[] = {
	0.0141  ,  0.0300  ,  0.0720  ,  0.1245  ,  0.1674  ,  0.1838  ,  0.1674  ,  0.1245  ,  0.0720,
	0.0300  ,  0.0141
};

#define SIZE_ANGLE			sizeof(Hd)
#define SIZE_SPEED			sizeof(Hd)

extern volatile uint8_t flag_time_sample;
extern volatile uint32_t g_ul_ms_ticks;

#endif /* TIMER_H_ */