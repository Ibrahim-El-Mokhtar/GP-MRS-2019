#ifndef QUADRATURE_ENCODER_H_
#define QUADRATURE_ENCODER_H_
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
#include "i2c_tm4c123.h"

extern void QEI0IntHandler(void);
extern void QEI1IntHandler(void);

void QEIInit(uint32_t QEI_BASE ,uint32_t numberOfclockTicks, uint32_t numberOfticksPerEncoderRevolution)
{
    if (QEI_BASE == QEI0_BASE )
    {
            SysCtlPeripheralEnable (SYSCTL_PERIPH_QEI0);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
            SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){}
            GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
            GPIOPinConfigure(GPIO_PD6_PHA0);
            GPIOPinConfigure(GPIO_PD7_PHB0);
            QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B //capture both edges
                                     | QEI_CONFIG_NO_RESET //no reset, no index to reset position
                                     | QEI_CONFIG_QUADRATURE // use the 2 phases
                                     | QEI_CONFIG_NO_SWAP, // no swap
                                     numberOfticksPerEncoderRevolution);//490

            QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1, numberOfclockTicks); //3200000

            QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_2);

            //enable velocity
            QEIVelocityEnable(QEI0_BASE);
            //enable filter
            QEIFilterEnable(QEI0_BASE);

            //enable encoder
            QEIEnable(QEI0_BASE);

            QEIIntRegister(QEI0_BASE,(*QEI0IntHandler));
            QEIIntEnable(QEI0_BASE, QEI_INTTIMER );
            IntRegister(INT_QEI0,  (*QEI0IntHandler));
            IntPrioritySet(INT_QEI0, 1);
            IntEnable(INT_QEI0);
    }

    else if (QEI_BASE == QEI1_BASE )
    {
            SysCtlPeripheralEnable (SYSCTL_PERIPH_QEI1);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1)){}
            SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
            while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
            GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
            GPIOPinConfigure(GPIO_PC5_PHA1);
            GPIOPinConfigure(GPIO_PC6_PHB1);
            QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B//capture both edges
                                     | QEI_CONFIG_NO_RESET //no reset, no index to reset position
                                     | QEI_CONFIG_QUADRATURE // use the 2 phases
                                     | QEI_CONFIG_NO_SWAP, // no swap
                                     numberOfticksPerEncoderRevolution);//490

            QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1, numberOfclockTicks); //3200000

            QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_2);

            //enable velocity
            QEIVelocityEnable(QEI1_BASE);
            //enable filter
            QEIFilterEnable(QEI1_BASE);


            //enable encoder
            QEIEnable(QEI1_BASE);

            QEIIntRegister(QEI1_BASE,(*QEI1IntHandler));
            QEIIntEnable(QEI1_BASE, QEI_INTTIMER );
            IntRegister(INT_QEI1,  (*QEI1IntHandler));
            IntPrioritySet(INT_QEI1, 1);
            IntEnable(INT_QEI1);



    }


}
#endif
