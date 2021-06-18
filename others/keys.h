/*
 * keys.h
 *
 *  Created on: 2018Äê3ÔÂ13ÈÕ
 *      Author: donghao
 */

#ifndef OTHERS_KEYS_H_
#define OTHERS_KEYS_H_

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

#define PWM_STOP (GpioDataRegs.GPADAT.bit.GPIO13==0)
#define PWM_RUN (GpioDataRegs.GPADAT.bit.GPIO14==0)

#define SCOPE_TOGGLE   {GpioDataRegs.GPBTOGGLE.bit.GPIO61=1;}
#define SCOPE_PU    {GpioDataRegs.GPBSET.bit.GPIO61=1;}
#define SCOPE_PD    {GpioDataRegs.GPBCLEAR.bit.GPIO61=1;}

void KeyGpioInit(void);

#endif /* OTHERS_KEYS_H_ */
