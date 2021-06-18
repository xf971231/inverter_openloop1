/*
 * keys.c
 *
 *  Created on: 2018Äê3ÔÂ13ÈÕ
 *      Author: donghao
 */

#include "keys.h"

void KeyGpioInit(void){
    EALLOW ;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 0 ; // Set as input ;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0 ; // Set as common function

    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0 ; // Set as input ;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0 ; // Set as common function

    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1 ; // Set as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0 ; // Set as common function

    EDIS ;
}

