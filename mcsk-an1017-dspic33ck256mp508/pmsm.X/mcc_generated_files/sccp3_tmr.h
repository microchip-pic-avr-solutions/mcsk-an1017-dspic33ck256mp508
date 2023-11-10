/**
  SCCP3 Generated Driver API Header File 

  @Company
    Microchip Technology Inc.

  @File Name
    sccp3.h

  @Summary
    This is the generated header file for the SCCP3 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides implementations for driver APIs for SCCP3. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.166.0
        Device            :  dsPIC33CK256MP508
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.41
        MPLAB             :  MPLAB X v5.30
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

#ifndef _SCCP3_TMR_H
#define _SCCP3_TMR_H

/**
  Section: Included Files
*/

#include <xc.h> 
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
/**
  Section: Interface Routines
*/
/**
  @Summary
    Initializes hardware and data for the given instance of the TMR module

  @Description
    This routine initializes hardware for the instance of the TMR module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  @Param
    None.

  @Returns
    None

  @Comment

 
  @Example
    <code>
    bool statusTimer1;
    uint16_t period;
    uint16_t value;

    period = 0x20;

    SCCP3_TMR_Initializer();

    SCCP3_TMR_Period16BitSet(period);

    if((value = SCCP3_TMR_Period16BitGet())== period)
    {
        SCCP3_TMR_Start();
    }

    while(1)
    {
        SCCP3_TMR_Tasks();
        if( (statusTimer1 = SCCP3_TMR_IsElapsed()) == true)
        {
            SCCP3_TMR_Stop();
        }
    }
    </code>
*/
void SCCP3_TMR_Initialize (void);

/**
  @Summary
    Starts the TMR

  @Description
    This routine starts the TMR

  @Param
    None.

  @Returns
    None

  @Example
    Refer to the example of SCCP3_Initializer();
*/

void SCCP3_TMR_Start( void );
/**
  @Summary
    Stops the TMR

  @Description
    This routine stops the TMR

  @Param
    None.

  @Returns
    None

  @Example
    Refer to the example of SCCP3_Initializer();
*/

void SCCP3_TMR_Stop( void );

/**
  @Summary
    Callback for SCCP3 Timer 32 bit.

  @Description
    This routine is callback for SCCP3 Timer 32 bit

  @Param
    None.

  @Returns
    None
 
  @Example 
	Refer to SCCP3_Initialize(); for an example
*/
void SCCP3_TMR_Timer32CallBack(void);

/**
  void Tasks( void )
 
  @Summary
    Maintains the driver's secondary timer state machine in a polled manner

  @Description
    This routine is used to maintain the driver's internal secondary timer state
    machine.This routine is called when the state of the secondary timer needs to be
    maintained in a polled manner.

  @Param
    None.

  @Returns
    None
*/
void SCCP3_TMR_Timer32Tasks( void );

/**
  @Summary
    Updates 32-bit mccp value

  @Description
    This routine updates 32-bit mccp value

  @Param
    None.

  @Returns
    None

  @Example
    Refer to the example of SCCP3_TMR_Initializer();
*/

void SCCP3_TMR_Period32BitSet( uint32_t value );

/**
  @Summary
    Provides the mccp 32-bit period value

  @Description
    This routine provides the mccp 32-bit period value

  @Param
    None.

  @Returns
    Timer 32-bit period value

  @Example
    Refer to the example of SCCP3_TMR_Initializer();
*/

uint32_t SCCP3_TMR_Period32BitGet( void );

/**
  @Summary
    Updates the mccp's 32-bit value

  @Description
    This routine updates the mccp's 32-bit value

  @Param
    None.

  @Returns
    None  @Example
    <code>
    uint32_t value=0xF0F0F0F0;

    SCCP3_TMR_Counter32BitSet(value));

    while(1)
    {
        SCCP3_TMR_Tasks();
        if( (value == SCCP3_TMR_Counter32BitGet()))
        {
            SCCP3_TMR_Stop();
        }
    }
    </code>
*/

void SCCP3_TMR_Counter32BitSet( uint32_t value );

/**
  @Summary
    Provides 32-bit current counter value

  @Description
    This routine provides 32-bit current counter value

  @Param
    None.

  @Returns
    32-bit current counter value

  @Example
    Refer to the example of SCCP3_TMR_Counter32BitSet();
*/

uint32_t SCCP3_TMR_Counter32BitGet( void );

/**
  @Summary
    Returns the elapsed status of the mccp

  @Description
    This routine returns the elapsed status of the mccp

  @Param
    None.

  @Returns
    bool - Elapsed status of the mccp.

  @Example
    Refer to the example of SCCP3_TMR_Initializer();
*/

bool SCCP3_TMR_Timer32ElapsedThenClear(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif //_SCCP3_H

/**
 End of File
*/
