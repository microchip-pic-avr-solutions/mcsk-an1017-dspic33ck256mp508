/*******************************************************************************
 *  Hall Sensor-based control of PMSM Drive Configuration, Constants,Routine Header File
 * 
 *  File Name:
 *    pmsm_main.h
 *   
 *  Summary:
 *    This header file lists PMSM drive Configuration related functions and
 *    definitions.
 *
 *  Description:
 *    Definitions in the file are for dsPIC33CK256MP508 External OP-AMP PIM
 *    plugged onto Motor Control Development board from Microchip.
 */
/*******************************************************************************/
/*******************************************************************************
 * Copyright (c) 2019 released Microchip Technology Inc.  All rights reserved.
 *
 * SOFTWARE LICENSE AGREEMENT:
 * 
 * Microchip Technology Incorporated ("Microchip") retains all ownership and
 * intellectual property rights in the code accompanying this message and in all
 * derivatives hereto.  You may use this code, and any derivatives created by
 * any person or entity by or on your behalf, exclusively with Microchip's
 * proprietary products.  Your acceptance and/or use of this code constitutes
 * agreement to the terms and conditions of this notice.
 *
 * CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
 * WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
 * PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
 * WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
 * STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
 * FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
 * HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
 * THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
 * MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
 * SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
 * HAVE THIS CODE DEVELOPED.
 *
 * You agree that you are solely responsible for testing the code and
 * determining its suitability.  Microchip has no obligation to modify, test,
 * certify, or support the code.
 *
 *******************************************************************************/
#ifndef pmsm_MAIN_H
#define	pmsm_MAIN_H

// *****************************************************************************
// Section: Included Files
// *****************************************************************************
#include <stdint.h>
//#include "../pmsm.X/library-motor/motor_control_declarations.h"
#include "../library/library-motor/motor_control_declarations.h"
#include "../library/library-motor/motor_control_types.h"

#include "peripherals.h"
#include "measure.h"

// *****************************************************************************
// Section: MODE OF OPERATION
// *****************************************************************************  
#define OPENLOOP

/* GLOBAL DEFINES FOR FUNCTIONING MODE */
/* undefine for negation */
#define PHASE_ADVANCE    // for extended speed ranges this should be defined    
//#define OCDETECTION
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/** Motor Ratings (from Name Plate Details or Datasheet)*/
// *****************************************************************************
    
#define POLEPAIRS		    2      // Number of pole pairs    
#define SECTOR              6      // Number of Electrical Sectors in Motor
#define MAX_MOTORSPEED      4500   // Specify the maximum speed in rpm of motor
#define NOMINAL_MOTORSPEED  2900   // Specify the nominal speed in rpm of motor
#define MIN_OL_MOTORSPEED   600    // Specify the min openloop speed in rpm of motor
#define MIN_CL_MOTORSPEED   1000   // Specify the min closedloop speed in rpm of motor 
#define MAX_MOTORCURRENT    1      // Max-Ampere Rating of Motor
#define MAX_BOARDCURRENT    22     // Max-Ampere Rating of LVMC Development Board
#define MIN_CTRL_OUTPUT     1700   //Minimum control output for openloop control
#define MAX_CTRL_OUTPUT     32760  //Maximum control output for openloop control

// ***************************************************************************** 

// These Phase values represent the base Phase value of the sinewave for each
// one of the sectors (each sector is a translation of the hall effect sensors
// reading 
#define PHASE_ZERO 	0
#define PHASE_ONE	((PHASE_ZERO + 65536/6) % 65536) //10922
#define PHASE_TWO	((PHASE_ONE + 65536/6) % 65536)  //21845
#define PHASE_THREE	((PHASE_TWO + 65536/6) % 65536)  //32768
#define PHASE_FOUR	((PHASE_THREE + 65536/6) % 65536)//43691
#define PHASE_FIVE	((PHASE_FOUR + 65536/6) % 65536) //54613

#define TIMER_PRESCALER     64
// Period Calculation
// Period = (FCY / TIMER_PRESCALE) / (RPM * NO_POLEPAIRS )/10
#define MAXPERIOD	(unsigned long)(((float)FCY / (float)TIMER_PRESCALER) / (float)((MIN_OL_MOTORSPEED * POLEPAIRS)/10))	
#define MINPERIOD	(unsigned long)(((float)FCY / (float)TIMER_PRESCALER) / (float)((MAX_MOTORSPEED * POLEPAIRS)/10))
//Maximum number of ticks in lowest speed for the counter used
#define PERIOD_CONSTANT  (unsigned long)((float)MAXPERIOD *(float)MIN_OL_MOTORSPEED) 

//(FCY/(TIMER_PRESCALER*FPWM)*(65536/6))
#define PHASE_INC_CALC  (unsigned long)((float)FCY/((float)(TIMER_PRESCALER)*(float)(PWMFREQUENCY_HZ))*(float)(65536/6))    

// In the sinewave generation algorithm we need an offset to be added to the
// pointer when energizing the motor in CCW. This is done to compensate an
// asymmetry of the sinewave
#define PHASE_OFFSET_CW     43688
#define PHASE_OFFSET_CCW    21988


#define OC_COUNTER 10000
#define OC_DETECT DAC1CONLbits.CMPSTAT

// This value represents the maximum allowed phase advance in electrical degrees. 
//Set a value from 0 to 60. This value will be used to calculate phase advance 
//only if PHASE_ADVANCE is defined
#define MAX_PH_ADV_ANGLE  10 // The maximum phase advance angle
#define SPEED_DIFF (MAX_MOTORSPEED - NOMINAL_MOTORSPEED) // The diffrence between maximum and nominal speed for phase advance
#define PHASE_ADVANCE_CONSTANT Q15((float) MAX_PH_ADV_ANGLE /SPEED_DIFF) // Constant term used for phase advance equation

/* maximum phase advance formula */
//#define PH_ADV 		(int)(((float)PH_ADV_DEG / 360.0) * 65536.0) 

// *****************************************************************************
#define REVERSE_DROP_SPEED   100   // Speed to be reduced in rpm before reversing the direction
// ***************************************************************************** 
/** Constants for Mathematical Computation */
#define TICKS            FCY/(TIMER_PRESCALER)

/**  SPEED MULTIPLIER CALCULATION = ((FCY*60)/(TIMER_PRESCALER*POLEPAIRS))  */
#define SPEED_MULTI     (unsigned long)(((float)FCY/(float)(TIMER_PRESCALER*POLEPAIRS))*(float)60) 

#define REV_SPEED_LIMIT   (unsigned long) ((float)(SPEED_MULTI)/(float)(REVERSE_DROP_SPEED*6))

//******************************************************************************
/** Velocity Control Loop Coefficients */
#define SPEEDCNTR_PTERM        Q15(0.2)
#define SPEEDCNTR_ITERM        Q15(0.002)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       Q15(0.999)
#define SPEEDCNTR_OUTMIN       Q15(0.0)

// *****************************************************************************
/** Moving Average - No of Samples*/
#define PERIOD_MOVING_AVG_FILTER_SCALE     4
#define PERIOD_MOVING_AVG_FILTER_SIZE       (uint16_t)(1 << PERIOD_MOVING_AVG_FILTER_SCALE) 
#define SPEED_MOVING_AVG_FILTER_SCALE      4
#define SPEED_MOVING_AVG_FILTER_SIZE       (uint16_t)(1 << SPEED_MOVING_AVG_FILTER_SCALE) 
// ***************************************************************************** 
//HANDLE TO CHANGE DEFAULT SPIN DIRECTION
#define RUN_DIRECTION   true         //boolean logic > true = 1, false = 0; CW

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// ***************************************************************************** 
void phaseAdvanceDegree(void);
void MCAPP_InitMovingAvgPeriod(void); //Initialize Moving Average Period Filter
void MCAPP_CalcMovingAvgPeriod(uint16_t instPeriod); //Calculates Period by Moving Average
void MCAPP_CalcMovingAvgSpeed(int16_t instSpeed); //Calculates Speed by Moving Average
void MCAPP_StateMachine(void); //Function to run the State Machine 
void MCAPP_CheckHallUpdatePWM(void); //Read Hall Port Data and update PWM switching
void MCAPP_InitControlParameters(void);

// Constants used for properly energizing the motor depending on the 
// rotor's position
const int PhaseValuesCC[8] = {0, PHASE_FOUR, PHASE_TWO, PHASE_THREE, PHASE_ZERO, PHASE_FIVE, PHASE_ONE, 0};

const int PhaseValues[8] = {0, PHASE_ZERO, PHASE_FOUR, PHASE_FIVE, PHASE_TWO, PHASE_ONE, PHASE_THREE, 0};

// This variable is incremented by the ADC interrupt
// in order to generate a proper sinewave. Its value
// is incremented by a value of PhaseInc, which
// represents the frequency of the generated sinewave
signed int Phase; 

// Delta increments of the Phase variable, calculated and used
// in the adc interrupt (each 50 us)
uint16_t PhaseInc;

// Used for extending motor speed range. This value
// is added directly to the parameters passed to the
// SVM function (the sine wave generation subroutine)
signed int PhaseAdvance; 

// Variables containing the Period of a sector or 1/6 
//of an electrical cycle, which is an  interrupt each 
//edge of one of the hall sensor input
uint32_t PastCapture, ActualCapture;
uint16_t Period, AvgPeriod;

// Controller output, used as a voltage output,
int ControlOutput = 0;

// variable for phase advance in degree
//int PH_ADV_DEG = 0; 

// Used as a temporal variable to perform a multiply operation in assembly
int PH_ADV = 0; 

// Open loop desired pot speed 
int openLoopDesiredSpeed = 0;


// *****************************************************************************
// Section: Enums, Structures
// *****************************************************************************

typedef struct 
{
    uint16_t index;
    int16_t buffer[PERIOD_MOVING_AVG_FILTER_SIZE];
    int32_t sum;
    uint16_t avg;
} MCAPP_PERIOD_MOVING_AVG_T;

typedef struct 
{
    uint32_t speedAcc;
    uint32_t calculatedSpeed;
    uint16_t speedValue;

    uint16_t index;
    int16_t buffer[SPEED_MOVING_AVG_FILTER_SIZE];
    int32_t sum;
    int16_t avg;
} MCAPP_SPEED_MOVING_AVG_T;

typedef struct 
{
    uint16_t runMotor;
    uint16_t changeDirection;
    uint16_t state;
    uint16_t runCmd;
    uint16_t runDirection;
    uint16_t desiredSpeed;
    uint16_t sector;
    int16_t dutyCycle;
    int16_t pwmPeriod;

    MCAPP_MEASURE_T analogInputs;
    MC_PIPARMIN_T piInputCurrent;
    MC_PIPARMOUT_T piOutputCurrent;
    MC_PIPARMIN_T piInputSpeed;
    MC_PIPARMOUT_T piOutputSpeed;
    MCAPP_SPEED_MOVING_AVG_T movingAvgFilterSpeed;
    MCAPP_PERIOD_MOVING_AVG_T movingAvgFilterPeriod;
} MCAPP_DATA_T;

typedef struct 
{
    unsigned MotorRunning : 1; // This bit is 1 if motor running
    unsigned unused : 15;
} Flags;

typedef enum 
{
    MCAPP_INIT = 0, /* Initialize Run time parameters */
    MCAPP_CMD_WAIT = 1, /* Wait for Run command */
    MCAPP_OFFSET = 2, /* Measure current offsets */
    MCAPP_RUN = 3, /* Run the motor */
    MCAPP_CHANGE_DIRECTION = 4, /* Change motor running direction */
    MCAPP_STOP = 5, /* Stop the motor */
    MCAPP_FAULT = 6, /* Motor is in Fault mode */
} MCAPP_STATE_T;

//----------------------------------------------------------------------
// 	Linix 36ZWN24 Motor Terminals |
// -----------------------|---------------------------------
//	Phase White  ---------|-- PHA
//	Phase Blue   ---------|-- PHB
//	Phase Green  ---------|-- PHC
//	Hall White   ---------|-- HA
//	Hall Blue    ---------|-- HB
//	Hall Green   ---------|-- HC
//  Hall Red     ---------|-- 5V
//  Hall Black   ---------|-- GND

#endif	/** pmsm_MAIN_H */