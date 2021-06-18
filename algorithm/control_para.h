/*
 * control_para.h
 *
 *  Created on: 2018Äê3ÔÂ12ÈÕ
 *      Author: donghao
 */

#ifndef CONTROLLER_CONTROL_PARA_H_
#define CONTROLLER_CONTROL_PARA_H_

/***************************************************************************************************/
// #define VOLTAGE_REF 80.0f
#define CURRENT_REF 10.0f
// #define FREQ_G 50.0f

#define KP_PLL 0.2 // 0.08
#define KI_PLL 0.6 // 0.18
#define K_GAIN_PLL_SOGI 1.4142135623730950488016887242097
// #define TS_PLL_TEN_KHZ 1e-4

#define TS_TWENTY_KHZ 5.04e-5
// #define TS_TWENTY_KHZ 4.95e-5

#define K_GAIN_FLL 1.4142135623730950488016887242097
#define GAMMA_FLL 0.5

#define K_FEEDFORWARD 0

#define C_CTRL_KP 20.0f
#define C_CTRL_KI 15000.0f
#define C_CTRL_KD 25.0f

#define RC_LEAD_STEPS 11
#define RC_Q_COEFF 0.9
#define RC_K_RC 1.0

extern const float num_filter[3] ;
extern const float den_filter[3] ;
extern const int order_filter ;

#endif /* CONTROLLER_CONTROL_PARA_H_ */
