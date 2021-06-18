#ifndef WDVRC_H
#define WDVRC_H

#include "../../filter/filter.h"
#include "math.h"

#define MAX_STEPS_FOR_BUFFER 410
// #define STEPS_AT_BASE_FREQ 400
// #define SAMPLE_FREQUENCY 20000
//#define SAMPLE_FREQUENCY 20202.0202
//#define SAMPLE_FREQUENCY 20161.29032
//#define SAMPLE_FREQUENCY 20120.72435 // 49.7Hz
//#define SAMPLE_FREQUENCY 20080.32129 // 49.8Hz
//#define SAMPLE_FREQUENCY 20040.08016
//#define SAMPLE_FREQUENCY 20000.0
//#define SAMPLE_FREQUENCY 19960.07984
//#define SAMPLE_FREQUENCY 19920.31873
//#define SAMPLE_FREQUENCY 19880.71571
#define SAMPLE_FREQUENCY 19841.26984
//#define SAMPLE_FREQUENCY 19801.9802

#define DEBUG 0

typedef struct fir_rc_structure{
    float q_output_buffer[MAX_STEPS_FOR_BUFFER] ;

    float delay_prev_buffer[MAX_STEPS_FOR_BUFFER]  ;

    float q_input_buffer[MAX_STEPS_FOR_BUFFER] ;

    int q_input_index ;
    int q_output_index ;
    int delay_prev_index ;

    int lead_steps ;
    filter_s * pfs ;
    float q_main ;
    float q_lr ;

    float k_rc ;
    float w_dr ; // delay 1 more step ; e.g. 202
    float w_dl ; // delay 1 less step ; e.g. 201
}fir_rc_struct ;

int index_move(int cur_index, int steps_to_move) ;

#if DEBUG

void init_fir_rc_debug() ;
float calc_fir_rc_debug(float freq_grid, float error);

#else

void init_fir_rc(fir_rc_struct * p_dvrc_s, filter_s * p_f_s, float q_main,  float k_rc, int steps_lead);
float calc_fir_rc(fir_rc_struct * p_dvrc_s, float freq_grid, float error);

#endif

#endif
