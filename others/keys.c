/*
 * keys.c
 *
 *  Created on: 2018年3月13日
 *      Author: donghao
 */

#include "keys.h"

void KeyGpioInit(void){
    EALLOW ;
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0 ; // Set as input ;
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0 ; // Set as common function

    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0 ; // Set as input ;
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0 ; // Set as common function

    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0 ; // Set as input ;
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0 ; // Set as common function

    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0 ; // Set as input ;
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0 ; // Set as common function

    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1 ; //
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0 ; // GPIO17作为Scope Trigger位使用

    EDIS ;
//    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1 ;
//
//    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0 ;
//
//    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0 ;
//    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0 ;
//
//    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0 ;
//    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0 ;

}

//int KeyScan(void){
//
//
//    if(ROW_READ_0){
//        return 1 ;
//    }
//    if(ROW_READ_1){
//        return 1 ;
//    }
//    if(ROW_READ_2){
//        return 1 ;
//    }
//    if(ROW_READ_3){
//        return 1 ;
//    }
//
//    return 0 ;
//    COL_SET_0 ;
//    COL_SET_1 ;
//    COL_SET_2 ;
//
//}
