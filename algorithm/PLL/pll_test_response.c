#include "stdio.h"
#include "pll_sogi.h"
#include "math.h"

void main()
{
    pll_sogi_s pll_s ;
    init_pll_sogi(&pll_s, 0.5, 5.0, 5e-5) ;
    float ts = 5e-5, t = 0, phase = 0;
    FILE * fp = fopen("response.txt", "w") ;

    for(int i = 0 ; i < 2e4 ; i ++)  {
        t = (i+1) * ts ;
        phase += DOUBLE_PI_PLL_SOGI * 49 * ts ;
        calc_pll_sogi(&pll_s, 311 * sinf(phase) ) ;
        fprintf(fp, "%f,%f\r\n", t, pll_s.angle_freq/DOUBLE_PI_PLL_SOGI) ;
        if(phase > DOUBLE_PI_PLL_SOGI ){
            phase -= DOUBLE_PI_PLL_SOGI ;
        }
    }
    for(int i = 2e4 ; i < 5e4 ; i ++ ){
        t = (i+1) * ts ; ;
        phase += DOUBLE_PI_PLL_SOGI * 51 * ts ;
        calc_pll_sogi(&pll_s, 311 * sinf(phase) ) ;
        fprintf(fp, "%f,%f\r\n", t, pll_s.angle_freq/DOUBLE_PI_PLL_SOGI) ;
        if(phase > DOUBLE_PI_PLL_SOGI ){
            phase -= DOUBLE_PI_PLL_SOGI ;
        }
    }
    fclose(fp) ;
}