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

#define  KEY_CLOSELOOP_CONTROL_DISABLED  (GpioDataRegs.GPBDAT.bit.GPIO50==0)
#define  KEY_CLOSELOOP_CONTROL_ENABLED  (GpioDataRegs.GPBDAT.bit.GPIO51==0)
#define  KEY_CURRENT_CHANGE  (GpioDataRegs.GPBDAT.bit.GPIO50==0)

#define SCOPE_TOGGLE   {GpioDataRegs.GPATOGGLE.bit.GPIO17=1;}
#define SCOPE_PU    {GpioDataRegs.GPASET.bit.GPIO17=1;}
#define SCOPE_PD    {GpioDataRegs.GPACLEAR.bit.GPIO17=1;}

void KeyGpioInit(void);

#endif /* OTHERS_KEYS_H_ */
