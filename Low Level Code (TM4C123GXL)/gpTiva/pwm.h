/*
 * pwm.h
 *
 *  Created on: Mar 1, 2019
 *      Author: john
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/i2c.h"
#include "uartstdio.h"
#include "delay.h"
#define pwmFrequency 32000

void pwmInit(uint32_t PWM_BASE,uint32_t PWM_GEN )
{
     SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
     while(! SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){}


    if ((PWM_BASE == PWM0_BASE )& (PWM_GEN == PWM_GEN_2 ))
    {
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}
        UARTprintf("WTFFFFFFFFFFFF");

        GPIOPinConfigure(GPIO_PE4_M0PWM4); //PB6
        GPIOPinConfigure(GPIO_PE5_M0PWM5); //PB7
        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
        //    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
        //Counting mode (down up/down)
        PWMGenConfigure(PWM0_BASE, PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        //ACTIONS ON FLAGS (zero load CMPA-up/down CMPB-up/down)
        //    PWM0_0_GENA_R |=
        //                   (0x02<<2) // Action on zero drive pwmALow
        //                  |(0x03<<6); // Action on compare match A down drive pwmAHigh

        // In this case you get: (1 / 1000Hz) * 16MHz = 16000 cycles.  Note that
        // the maximum period you can set is 2^16.
        //20000
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, pwmFrequency );
        //  compare value ... AKA duty cycle
        //  PWM0_0_CMPA_R = 8000;
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 10000);

        // motor(signal) is high between 20000 and the next value
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwmFrequency -1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwmFrequency -1);


        PWMGenEnable(PWM0_BASE, PWM_GEN_2);

        PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT|PWM_OUT_5_BIT, true);
    }

    if ((PWM_BASE == PWM0_BASE ) & (PWM_GEN == PWM_GEN_1 ))
        {
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
            GPIOPinConfigure(GPIO_PB4_M0PWM2); //PB6
            GPIOPinConfigure(GPIO_PB5_M0PWM3); //PB7
            GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
            GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
            //    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
            //Counting mode (down up/down)

            PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
            //ACTIONS ON FLAGS (zero load CMPA-up/down CMPB-up/down)
            //    PWM0_0_GENA_R |=
            //                   (0x02<<2) // Action on zero drive pwmALow
            //                  |(0x03<<6); // Action on compare match A down drive pwmAHigh

            // In this case you get: (1 / 1000Hz) * 16MHz = 16000 cycles.  Note that
            // the maximum period you can set is 2^16.
            //
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmFrequency);
            //  compare value ... AKA duty cycle
            //  PWM0_0_CMPA_R = 8000;

            //initializing LOW
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwmFrequency -1);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwmFrequency -1);


            PWMGenEnable(PWM0_BASE, PWM_GEN_1);

            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT,true);

        }

}

void pwmMotorRight(uint32_t forward_1 ,uint32_t backward_0){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, backward_0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, forward_1);
}

void pwmMotorLeft(uint32_t forward_3 ,uint32_t backward_2){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, backward_2);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, forward_3);
}

#endif /* PWM_H_ */
