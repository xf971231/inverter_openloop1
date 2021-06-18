/** Inverter main program
 *  From Lab209 Prof. YongQiang Ye
 *  by Donghao@NUAA
 *  2017.12.15
 */

#include "main.h"

int flag_PWM = 0;;
/***********************************************************************************************************************
                                                         * main
 **********************************************************************************************************************/
int main(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

#if USE_PWM
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    PWM_Init(10000) ;
    IER|=M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1=1;
    PieCtrlRegs.PIEIER3.bit.INTx2=1;
    EINT;
    ERTM;
#endif

    KeyGpioInit();
    RelayGpioInit();

    for(;;)
    {
        if(flag_timer2_updated)
        {
            flag_timer2_updated = 0 ;
//            debug_scope_count ++ ;
//            debug_scope_count %= 400 ;
//            debug_scope_1[debug_scope_count] =  frequency_grid_Hz ;
            if(PWM_RUN)
            {
                flag_PWM = 1 ;

            }
            if(PWM_STOP)
            {
                flag_PWM = 0 ;

            }
            if(flag_PWM == 1)
            {
                Enable_PWM_RELAY();
            }
            else
            {
                Shutdown_PWM_RELAY();
            }
         }
    }
}


