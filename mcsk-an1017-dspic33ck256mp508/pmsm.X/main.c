/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.166.0
        Device            :  dsPIC33CK256MP508
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.41
        MPLAB 	          :  MPLAB X v5.30
 */

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

/**
  Section: Included Files
 */
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/pwm.h"
#include "mcc_generated_files/adc1.h"
#include "diagnostics/diagnostics.h"
#include "pmsm_main.h"
#include "general.h"
#include "measure.h"
#include "peripherals.h"
#include "board_service.h"
#include "svm.h"

// *****************************************************************************
// *****************************************************************************
// Section: MODE OF OPERATION
// *****************************************************************************
// *****************************************************************************  
MCAPP_DATA_T mcappData;
MCAPP_DUTYCYCLEOUT_T mcappDuty;
uint16_t V_M1, V_M2, V_M3, busCurrent = 0;
volatile uint16_t motorStallCounter = 0;
volatile bool startUp = 0;
uint16_t OvercurrentCounter;

void ChargeBootstrapCapacitors(void);
void ADC1_ISR(void);
void ForceCommutate(void);

/******************************************************************************
 * Description: The main function initializes the microcontroller port configurations
 *              and settings. It is the main program which also reads the button 
 *              data, that runs the mtor as well as reverses the direction of spin.
 *****************************************************************************/
int main(void) {
    SYSTEM_Initialize();
    DiagnosticsInit();
    ChargeBootstrapCapacitors();
    CN_Configure();
    ADC_Channel_Config();
    BoardServiceInit();
    HAL_MC1PhaseStateChangeMaxPeriodSet(PERIOD_CONSTANT);
    mcappData.runDirection = RUN_DIRECTION; //direction of spin
    LED1_SetHigh();
    
    while (1) 
    {
        DiagnosticsStepMain();
        BoardService();

        if (IsPressed_Button1()) 
        {
            if (mcappData.runCmd == 0) 
            {
                mcappData.runCmd = 1;
                LED2_SetHigh();
                mcappData.state = MCAPP_INIT;
                startUp = 1;
            } 
            else 
            {
                mcappData.runCmd = 0;
                LED2_SetLow();
            }
        }
        
        if (IsPressed_Button2() && mcappData.changeDirection == 0) 
        {
            mcappData.changeDirection = 1;
        }

    }
}

/******************************************************************************
 * Description: The MCAPP_CheckHallUpdatePWM function reads the Hall Sensor data
 *              and generates the switching pattern based on its sector.
 *****************************************************************************/
void MCAPP_CheckHallUpdatePWM(void) 
{
    mcappData.sector = HAL_HallValueRead();

    if (mcappData.runCmd == 1) 
    {
        PWM1_SwapOverrideEnableDataSet(0x0000);
        PWM2_SwapOverrideEnableDataSet(0x0000);
        PWM4_SwapOverrideEnableDataSet(0x0000);
    }
}

/******************************************************************************
 * Description: The ADCAN11 Interrupt operates at 20kHz. The analog data such as 
 *              the potentiometer voltage, Bus Current are read. It services
 *              the X2C Scope ISR, The MCAPP_StateMachine routines as well.
 *****************************************************************************/

void ADC1_ISR(void) 
{
    V_M1 = ADCBUF17; 
    V_M2 = ADCBUF23;
    V_M3 = ADCBUF22; 
    busCurrent = ADCBUF4;       
    /**  Routine to read the potentiometer value and the bus current value */
    HAL_MC1MotorInputsRead(&mcappData.analogInputs);
    /**  Routine to run the Motor Control State Machine */
    MCAPP_StateMachine();
    BoardServiceStepIsr();
    DiagnosticsStepIsr();
}



void phaseAdvanceDegree(void)
{
    #ifdef PHASE_ADVANCE
int16_t phaseAdvanceSpeed;
uint16_t phaseAdvanceStepDegree;
int16_t phaseAdvanceValue;
    // phaseAdvanceStepDegree = phaseAdvanceSpeed * PHASE_ADVANCE_CONSTANT
    phaseAdvanceSpeed = mcappData.desiredSpeed - NOMINAL_MOTORSPEED; 

    if (phaseAdvanceSpeed < 0)
        {
            phaseAdvanceStepDegree = 0;
        }
    else{

        phaseAdvanceStepDegree = __builtin_muluu(phaseAdvanceSpeed , PHASE_ADVANCE_CONSTANT)>>15;}

    phaseAdvanceValue = (int)(((float)phaseAdvanceStepDegree / 360.0) * 65536.0);
//    PH_ADV_DEG = phaseAdvanceStepDegree; // Calculated degree for the phase advance speed
    PH_ADV = phaseAdvanceValue; // Used as a temporal variable to perform a multiply operation in assembly
    
    #endif
}
/******************************************************************************
 * Description: The CND Interrupt is serviced at every hall signal transition. 
 *              This enables to calculate the speed based on the time taken for
 *              360 degree electrical rotation.
 *****************************************************************************/
void CNE_ISR(void) 
{
    motorStallCounter = 0;
    STALL_INDICATOR_BIT_SetLow();
    MCAPP_CheckHallUpdatePWM();
    ActualCapture = SCCP3_TMR_Counter32BitGet();
    Period = ActualCapture - PastCapture;
    if (ActualCapture < PastCapture) 
    {
        Period = PERIOD_CONSTANT - (PastCapture - ActualCapture);
    }
    PastCapture = ActualCapture;
    MCAPP_CalcMovingAvgSpeed(Period);
    if (Period < MINPERIOD) 
    {
        Period = MINPERIOD;
    } 
    else if (Period > MAXPERIOD) 
    {
        Period = MAXPERIOD;
    }

    if (mcappData.runDirection == 0) 
    {
        Phase = PhaseValuesCC[(mcappData.sector)] + PHASE_OFFSET_CCW;
    } 
    else 
    {
        Phase = PhaseValues[(mcappData.sector)] + PHASE_OFFSET_CW;
    } 
    
    phaseAdvanceDegree();

#ifdef PHASE_ADVANCE
    // Calculate Phase Advance Based on Actual Speed and PH_ADV define
    // The following assembly instruction perform the following formula
    // using fractional multiplication:
    // 
    // PhaseAdvance = PH_ADV * mcappData.movingAvgFilterSpeed.avg
    //

    register int a_reg asm("A");
    a_reg = __builtin_mpy(PH_ADV, mcappData.movingAvgFilterSpeed.avg, 0, 0, 0, 0, 0, 0);
    PhaseAdvance = __builtin_sac(a_reg, 0); 

#endif 
}

/******************************************************************************
 * Description: Force commutate executes when no hall state change is sensed. 
 * This function will identify the current hall state and load the corresponding 
 * phase values. 
 *****************************************************************************/
void ForceCommutate(void) 
{
    MCAPP_CheckHallUpdatePWM();
    if (mcappData.runDirection == 0) 
    {
        Phase = PhaseValuesCC[(mcappData.sector)] + PHASE_OFFSET_CCW;
    } 
    else 
    {
        Phase = PhaseValues[(mcappData.sector)] + PHASE_OFFSET_CW;
    }

}

/******************************************************************************
 * Description: The MCAPP_StateMachine describes the code flow and controls the
 *              pmsm motor based on its state. Thus enabling smooth control of motor
 *              and is easily understood by the user.
 *****************************************************************************/
void MCAPP_StateMachine(void) 
{
    switch (mcappData.state) 
    {
        case MCAPP_INIT:

            HAL_MC1PWMDisableOutputs();

            mcappData.runMotor = 0;
            mcappData.pwmPeriod = PWM_MasterPeriodRead();
            mcappData.dutyCycle = START_DUTY;
            mcappData.sector = 0;

            MCAPP_InitControlParameters();
            MCAPP_InitMovingAvgPeriod();
            mcappData.state = MCAPP_CMD_WAIT;
            OvercurrentCounter = 0;
            break;

        case MCAPP_CMD_WAIT:

            if (mcappData.runCmd == 1) 
            {
                mcappData.runMotor = 1;
                CN_PortEEnable();
                mcappData.state = MCAPP_RUN;
            }

            if (mcappData.changeDirection == 1) 
            {
                mcappData.state = MCAPP_CHANGE_DIRECTION;
            }
            break;

        case MCAPP_RUN:

            motorStallCounter++;

            if ((motorStallCounter % 200) == 0) 
            {
                ForceCommutate();
            }
            MCAPP_CheckHallUpdatePWM();
            MCAPP_CalcMovingAvgPeriod(Period);
            AvgPeriod = mcappData.movingAvgFilterPeriod.avg;
            if (AvgPeriod != 0)
                PhaseInc = __builtin_divud((long unsigned int) PHASE_INC_CALC, (unsigned int) AvgPeriod);
                      
#ifdef OPENLOOP
    #ifdef PHASE_ADVANCE
            mcappData.desiredSpeed = (int16_t) ((__builtin_mulss(ADCBUF_POT >> 1, (MAX_MOTORSPEED - MIN_OL_MOTORSPEED)) >> 15) + MIN_OL_MOTORSPEED);
    #else
            mcappData.desiredSpeed = (int16_t) ((__builtin_mulss(ADCBUF_POT >> 1, (NOMINAL_MOTORSPEED - MIN_OL_MOTORSPEED)) >> 15) + MIN_OL_MOTORSPEED); 
    #endif 
            ControlOutput = (int16_t) ((__builtin_mulss(mcappData.analogInputs.measurePot, (MAX_CTRL_OUTPUT - MIN_CTRL_OUTPUT)) >> 15) + MIN_CTRL_OUTPUT);
            openLoopDesiredSpeed = (int16_t) ((__builtin_mulss(ADCBUF_POT >> 1, (MAX_MOTORSPEED - MIN_OL_MOTORSPEED)) >> 15) + MIN_OL_MOTORSPEED );

#else
    #ifdef PHASE_ADVANCE
            mcappData.desiredSpeed = (int16_t) ((__builtin_mulss(ADCBUF_POT >> 1, (MAX_MOTORSPEED - MIN_CL_MOTORSPEED)) >> 15) + MIN_CL_MOTORSPEED);
    #else
            mcappData.desiredSpeed = (int16_t) ((__builtin_mulss(ADCBUF_POT >> 1, (NOMINAL_MOTORSPEED - MIN_CL_MOTORSPEED)) >> 15) + MIN_CL_MOTORSPEED); 
    #endif 
            mcappData.piInputSpeed.inMeasure = mcappData.movingAvgFilterSpeed.avg;
            mcappData.piInputSpeed.inReference = mcappData.desiredSpeed;

            MC_ControllerPIUpdate_Assembly(mcappData.piInputSpeed.inReference,
                    mcappData.piInputSpeed.inMeasure,
                    &mcappData.piInputSpeed.piState,
                    &mcappData.piOutputSpeed.out);
                ControlOutput = mcappData.piOutputSpeed.out;

#endif

            if (mcappData.runDirection == 0) 
            {
                Phase += PhaseInc;
#ifdef PHASE_ADVANCE  
               if (mcappData.desiredSpeed > NOMINAL_MOTORSPEED)
               
                {
                    Phase =  Phase + PhaseAdvance;
                }
                SVM(ControlOutput, Phase, &mcappDuty); 
#else  
                SVM(ControlOutput, Phase, &mcappDuty);             
#endif
            } 
            else 
            {
                Phase -= PhaseInc;
#ifdef PHASE_ADVANCE
                 if (mcappData.desiredSpeed > NOMINAL_MOTORSPEED)
                {
                    Phase =  Phase - PhaseAdvance;
                }
                
                SVM(ControlOutput, Phase, &mcappDuty); 
                 
#else
                SVM(ControlOutput, Phase, &mcappDuty);
#endif
            }
            HAL_MC1PWMSetDutyCycles(&mcappDuty);
            if (mcappData.changeDirection == 1) 
            {
                mcappData.dutyCycle = 0;
                HAL_MC1PWMSetDutyCyclesIdentical(mcappData.dutyCycle);
                HAL_MC1PWMDisableOutputs();
                mcappData.state = MCAPP_CHANGE_DIRECTION;
            }
            /*Count limit changed 900 to 5000 - sometimes stalling while starting in Phase Advance Mode*/
            if ((startUp == 0)&&(motorStallCounter >= 4900)) 
            {   
                motorStallCounter = 0;
                mcappData.state = MCAPP_STOP;
                STALL_INDICATOR_BIT_SetHigh();
            }
            startUp = 0;

            if (mcappData.runCmd == 0) 
            {
                mcappData.state = MCAPP_STOP;
            }

#ifdef OCDETECTION
            if (OC_DETECT) 
            {
                if (OvercurrentCounter > OC_COUNTER) 
                {
                    mcappData.state = MCAPP_STOP;
                    OvercurrentCounter = 0;
                }
                OvercurrentCounter++;
            }
#endif
            break;

        case MCAPP_CHANGE_DIRECTION:

            if (mcappData.changeDirection == 1) 
            {
                if (mcappData.runDirection == 0) 
                {
                    mcappData.runDirection = 1;
                } 
                else 
                {
                    mcappData.runDirection = 0;
                }
                mcappData.changeDirection = 0;
            }

            /*Wait for speed to reduce to a safe speed before reversing the direction*/
            if (mcappData.runCmd == 1) 
            {
                if (ActualCapture > REV_SPEED_LIMIT) 
                {
                    mcappData.state = MCAPP_RUN;
                    OvercurrentCounter = 0;
                    HAL_MC1PWMEnableOutputs();
                }
            } 
            else if (mcappData.runCmd == 0) 
            {
                mcappData.state = MCAPP_CMD_WAIT;
            }
            break;

        case MCAPP_STOP:

            mcappData.sector = 0;
            /** Set Duty Cycle */
            mcappData.dutyCycle = 0;
            HAL_MC1PWMSetDutyCyclesIdentical(mcappData.dutyCycle);
            HAL_MC1PWMDisableOutputs();

            mcappData.runCmd = 0;
            LED2_SetLow();
            break;
    }
}

void MCAPP_InitControlParameters(void) 
{
    /** PI - Speed Control */
    mcappData.piInputSpeed.piState.kp = SPEEDCNTR_PTERM;
    mcappData.piInputSpeed.piState.ki = SPEEDCNTR_ITERM;
    mcappData.piInputSpeed.piState.kc = SPEEDCNTR_CTERM;
    mcappData.piInputSpeed.piState.outMax = SPEEDCNTR_OUTMAX;
    mcappData.piInputSpeed.piState.outMin = SPEEDCNTR_OUTMIN;
    mcappData.piInputSpeed.piState.integrator = 0;
    mcappData.piOutputSpeed.out = 0;
}

/******************************************************************************
 * Description: The MCAPP_InitMovingAvgPeriod function initializes the current 
 *              array table that is being used to calculate the moving average,
 *              thereby eliminate the undesired response and variations during
 *              reset and restart of motor.
 *****************************************************************************/
void MCAPP_InitMovingAvgPeriod(void) 
{
    uint16_t i;

    for (i = 0; i < PERIOD_MOVING_AVG_FILTER_SIZE; i++) 
    {
        mcappData.movingAvgFilterPeriod.buffer[i] = 0;
    }

    mcappData.movingAvgFilterPeriod.index = 0;
    mcappData.movingAvgFilterPeriod.sum = 0;
    mcappData.movingAvgFilterPeriod.avg = 0;


    for (i = 0; i < SPEED_MOVING_AVG_FILTER_SIZE; i++) 
    {
        mcappData.movingAvgFilterSpeed.buffer[i] = 0;
    }

    mcappData.movingAvgFilterSpeed.index = 0;
    mcappData.movingAvgFilterSpeed.sum = 0;
    mcappData.movingAvgFilterSpeed.avg = 0;
}

/******************************************************************************
 * Description: The MCAPP_CalcMovingAvgPeriod function calculates the moving
 *              average period.
 *****************************************************************************/
void MCAPP_CalcMovingAvgPeriod(uint16_t instPeriod) 
{
    uint16_t i;

    mcappData.movingAvgFilterPeriod.buffer[mcappData.movingAvgFilterPeriod.index] = instPeriod;
    mcappData.movingAvgFilterPeriod.index++;
    if (mcappData.movingAvgFilterPeriod.index >= PERIOD_MOVING_AVG_FILTER_SIZE)
        mcappData.movingAvgFilterPeriod.index = 0;

    mcappData.movingAvgFilterPeriod.sum = 0;
    for (i = 0; i < PERIOD_MOVING_AVG_FILTER_SIZE; i++) 
    {
        mcappData.movingAvgFilterPeriod.sum = mcappData.movingAvgFilterPeriod.sum + mcappData.movingAvgFilterPeriod.buffer[i];
        mcappData.movingAvgFilterPeriod.avg = mcappData.movingAvgFilterPeriod.sum >> PERIOD_MOVING_AVG_FILTER_SCALE;
    }
}

/******************************************************************************
 * Description: The MCAPP_CalcMovingAvgSpeed function calculates the moving
 *              average of speed.
 *****************************************************************************/
void MCAPP_CalcMovingAvgSpeed(int16_t instSpeed) 
{
    uint16_t i;
    mcappData.movingAvgFilterSpeed.speedAcc = mcappData.movingAvgFilterSpeed.speedAcc + instSpeed;
    if (mcappData.sector == 6) 
    {
        mcappData.movingAvgFilterSpeed.speedValue = mcappData.movingAvgFilterSpeed.speedAcc;
        mcappData.movingAvgFilterSpeed.calculatedSpeed = (int16_t) (__builtin_divud(SPEED_MULTI, mcappData.movingAvgFilterSpeed.speedValue));
        mcappData.movingAvgFilterSpeed.speedAcc = 0;

        mcappData.movingAvgFilterSpeed.buffer[mcappData.movingAvgFilterSpeed.index] = mcappData.movingAvgFilterSpeed.calculatedSpeed;
        mcappData.movingAvgFilterSpeed.index++;
        if (mcappData.movingAvgFilterSpeed.index >= SPEED_MOVING_AVG_FILTER_SIZE)
            mcappData.movingAvgFilterSpeed.index = 0;

        mcappData.movingAvgFilterSpeed.sum = 0;
        for (i = 0; i < SPEED_MOVING_AVG_FILTER_SIZE; i++) 
        {
            mcappData.movingAvgFilterSpeed.sum = mcappData.movingAvgFilterSpeed.sum + mcappData.movingAvgFilterSpeed.buffer[i];
            mcappData.movingAvgFilterSpeed.avg = mcappData.movingAvgFilterSpeed.sum >> SPEED_MOVING_AVG_FILTER_SCALE;
        }
    }
}
void ChargeBootstrapCapacitors(void) 
{
    uint16_t i = BOOTSTRAP_CHARGING_COUNTS;
    uint16_t prevStatusCAHALF = 0, currStatusCAHALF = 0;
    uint16_t k = 0;

    HAL_MC1PWMDisableOutputs();

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialize the PWM duty cycle for charging
    PWM_PDC3 = LOOPTIME_TCY - (DEADTIME / 2 + 5);
    PWM_PDC2 = LOOPTIME_TCY - (DEADTIME / 2 + 5);
    PWM_PDC1 = LOOPTIME_TCY - (DEADTIME / 2 + 5);

    while (i) 
    {
        prevStatusCAHALF = currStatusCAHALF;
        currStatusCAHALF = PG1STATbits.CAHALF;
        if (prevStatusCAHALF != currStatusCAHALF) 
        {
            if (currStatusCAHALF == 0) 
            {
                i--;
                k++;
                if (i == (BOOTSTRAP_CHARGING_COUNTS - 50)) 
                {
                    // 0 = PWM generator provides data for PWM1L pin
                    PG1IOCONLbits.OVRENL = 0;
                } 
                else if (i == (BOOTSTRAP_CHARGING_COUNTS - 150)) 
                {
                    // 0 = PWM generator provides data for PWM2L pin
                    PG2IOCONLbits.OVRENL = 0;
                } 
                else if (i == (BOOTSTRAP_CHARGING_COUNTS - 250)) 
                {
                    // 0 = PWM generator provides data for PWM4L pin
                    PG4IOCONLbits.OVRENL = 0;
                }
                if (k > 25) {
                    if (PG4IOCONLbits.OVRENL == 0) 
                    {
                        if (PWM_PDC3 > 2) 
                        {
                            PWM_PDC3 -= 2;
                        } else 
                        {
                            PWM_PDC3 = 0;
                        }
                    }
                    if (PG2IOCONLbits.OVRENL == 0) 
                    {
                        if (PWM_PDC2 > 2) 
                        {
                            PWM_PDC2 -= 2;
                        } 
                        else 
                        {
                            PWM_PDC2 = 0;
                        }
                    }
                    if (PG1IOCONLbits.OVRENL == 0) 
                    {
                        if (PWM_PDC1 > 2) 
                        {
                            PWM_PDC1 -= 2;
                        } 
                        else 
                        {
                            PWM_PDC1 = 0;
                        }
                    }
                    k = 0;
                }
            }
        }
    }

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialize the PWM duty cycle for charging
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    PG4IOCONLbits.OVRENH = 0; // 0 = PWM generator provides data for PWM4H pin
    PG2IOCONLbits.OVRENH = 0; // 0 = PWM generator provides data for PWM2H pin
    PG1IOCONLbits.OVRENH = 0; // 0 = PWM generator provides data for PWM1H pin
}
/**
 End of File
 */