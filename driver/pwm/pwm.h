/*
 * pwm.h
 *
 *  Created on: 2017��12��18��
 *      Author: donghao
 */

#ifndef PWM_PWM_H_
#define PWM_PWM_H_

#include "setup.h"
#include "sensor.h"

#define MODULATION_UPPER_THRESHOLD 0.85f
#define MODULATION_LOWER_THRESHOLD -0.85f

#define MODULATION_UPPER_THRESHOLD_UINT 7400u
#define MODULATION_LOWER_THRESHOLD_UINT 450u

#define UNIPOLAR_MODULATION 1
#define UNIPOLAR_MODULATION_2 0

void PWM_Init(float freq);

#endif /* PWM_PWM_H_ */
