/*
 * File:   interrupt.c
 * Author: A20692
 *
 * Created on June 23, 2020, 10:35 AM
 */


#include <xc.h>
#include "peripherals.h"

void CN_Configure(void) 
{
    CNCONE = 0;
    /*  ON: Change Notification (CN) Control for PORTx On bit
        1 = CN is enabled
        0 = CN is disabled   */
    CNCONEbits.ON = 0;
    /*    CNSTYLE: Change Notification Style Selection bit
        1 = Edge style (detects edge transitions, bits are used for a CNE)
        0 = Mismatch style (detects change from last port read event)       */
    CNCONEbits.CNSTYLE = 0;

    CNEN0E = 0;
    CNEN0Ebits.CNEN0E8 = 1;
    CNEN0Ebits.CNEN0E9 = 1;
    CNEN0Ebits.CNEN0E10 = 1;

    _CNEIF = 0;
    _CNEIE = 1;
    _CNEIP = 7;
}

void __attribute__((interrupt, no_auto_psv)) _CNEInterrupt() 
{
    CNE_ISR();
    IFS4bits.CNEIF = 0;
}