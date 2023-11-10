/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef PERIPHERALS_H
#define	PERIPHERALS_H

#include <xc.h> // include processor files - each processor file is guarded. 

// OSCILLATOR Related Definitions
// Oscillator frequency (Hz) - 200,000,000 Hz
#define FOSC                    200000000UL
// Oscillator frequency (MHz) - 200MHz
#define FOSC_MHZ                200U     
// Instruction cycle frequency (Hz) - 100,000,000 Hz
#define FCY                     100000000UL
// Instruction cycle frequency (MHz) - 100 MHz
#define FCY_MHZ                 100U  

#define START_DUTY       MPER>>2     
#define MIN_DUTY         0
#define MAX_DUTY         MPER - 1 

/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         20000
/* Specify dead time in micro seconds */
#define DEADTIME_MICROSEC       1.0
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC            0.00005
/* Specify PWM Period in micro seconds */
#define LOOPTIME_MICROSEC       50
        
// Specify bootstrap charging time in Seconds (mention at least 10mSecs)
#define BOOTSTRAP_CHARGING_TIME_SECS 0.01
  
// Calculate Bootstrap charging time in number of PWM Half Cycles
#define BOOTSTRAP_CHARGING_COUNTS (uint16_t)((BOOTSTRAP_CHARGING_TIME_SECS/LOOPTIME_SEC )* 2) //400

#define DEADTIME               (uint16_t)(DEADTIME_MICROSEC*FOSC_MHZ)
// loop time in terms of PWM clock period
#define LOOPTIME_TCY            (uint16_t)(((LOOPTIME_MICROSEC*FOSC_MHZ)/2)-1)  //4998

/* Specify ADC Triggering Point w.r.t PWM Output for sensing Motor Currents */
#define ADC_SAMPLING_POINT      (FCY/PWMFREQUENCY_HZ)-2     //4998
#define ADCBUF_POT      ADCBUF11
#define ADCBUF_IBUS     ADCBUF4
#define PWM_PDC1  PG1DC
#define PWM_PDC2  PG2DC
#define PWM_PDC3  PG4DC

inline static void PWM1_SwapOverrideEnableDataSet(uint16_t data)
{
    PG1IOCONL = data & 0X7C00;
}
inline static void PWM2_SwapOverrideEnableDataSet(uint16_t data)
{
    PG2IOCONL = data & 0X7C00;
}
inline static void PWM4_SwapOverrideEnableDataSet(uint16_t data)
{
    PG4IOCONL = data & 0X7C00;
}
inline static uint16_t PWM_MasterPeriodRead(void)
{
    return MPER;
}

inline static void CN_PortEEnable(void){CNCONEbits.ON = 1;}

inline static void CN_PortEDisable(void){CNCONEbits.ON = 0;}

void CNE_ISR(void);
void CN_Configure(void);
void ADC1_ISR(void);

void CMP1_ISR(void);
void ADC_Channel_Config(void);
void OC_Enable(void);

#endif	/* PERIPHERALS_H */

