/*
 * relay.h
 *
 *  Created on: 2018��3��15��
 *      Author: donghao
 */

#ifndef OTHERS_RELAY_AND_PROTECT_H_
#define OTHERS_RELAY_AND_PROTECT_H_

#include "../include/DSP2833x_Device.h"
#include "../include/DSP2833x_Examples.h"
#include "../setup.h"

#include "ltrt.h"

#define PWM_EN {GpioDataRegs.GPASET.bit.GPIO24=1;}
#define PWM_DIS  {GpioDataRegs.GPACLEAR.bit.GPIO24=1;}

#define RELAY_OPEN {GpioDataRegs.GPASET.bit.GPIO22=1;}
#define RELAY_CLOSEUP {GpioDataRegs.GPACLEAR.bit.GPIO22=1;}

void RelayGpioInit(void) ;
void Shutdown_PWM_RELAY(void) ;
int ErrorDetected(void);
void Enable_PWM_RELAY(void);

#ifdef LONG_TIME_RUNNING_TEST
void MeasureCheck(void) ;
#endif

#endif /* OTHERS_RELAY_AND_PROTECT_H_ */
