/*
 * timer.h
 *
 *  Created on: Mar 2, 2019
 *      Author: john
 */

#ifndef TIMER_H_
#define TIMER_H_

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
#include "driverlib/timer.h"
#include "uartstdio.h"
#include "delay.h"
//#include "i2c_tm4c123.h"
#include "i2c_tivaware.h"
#include "quadrature_encoder.h"
#include "pwm.h"


void
timerInit(uint32_t TIMER_BASE, uint32_t config){
    //
    // Enable the Timer0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0))
    {
    }
    //
    // Configure TimerA as a half-width one-shot timer, and TimerB as a
    // half-width edge capture counter.
    //
    TimerConfigure(TIMER_BASE, config);
    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    TimerLoadSet(TIMER_BASE, TIMER_A, 16000000);

    //
    // Configure the counter (TimerB) to count both edges.
    //
    TimerControlEvent(TIMER_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

}

#endif /* TIMER_H_ */
