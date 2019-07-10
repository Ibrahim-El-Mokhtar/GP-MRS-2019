/*
 * i2c_tm4c123.h
 *
 *  Created on: Feb 23, 2019
 *      Author: john
 */

#ifndef I2C_TM4C123_H_
#define I2C_TM4C123_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
//#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
//#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
//#include "uartstdio.h"
extern void I2C0SlaveIntHandler(void);
enum {Run=1,Start=2,Stop=4,Ack=8}busOp;
enum {
    Busy=1,
    Error=2,
    AddressAck=4,
    DataAck=8,
    ArbitrationLost=16,
    Idle =32,
    BusBusy=64,
    ClockTimeOut=128
}masterbusState;
enum {
    slaveReceiveREQ =1,
    slaveTransmitREQ =2
}slavebusState;

void i2cMasterInit_R(void)
{
    //COMMON INIT
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
    GPIO_PORTB_AFSEL_R |= (GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTB_ODR_R |= 1<<3;
    GPIO_PORTB_PCTL_R |= ((3<<8) | 3<<12);

    //MASTER CONFIG
    I2C0_MCR_R |= 1<<4; // ACT AS MASTER
    I2C0_MTPR_R = 0X09; //TIME PERIOD ... CALCULATION IN DATASHEET
//    I2C0_MSA_R = (slave_addr <<1); // SLAVE ADDRESS WRITTEN IN MASTER
   //setting  send (0) or receive(1) bit
//    if (breceive ==false)
//        I2C0_MSA_R &= ~(1); // send
//    else
//        I2C0_MSA_R |= (1); // receive
}
//void i2cSlaveInit_R(uint32_t slave_addr , void (*pfnHandler)(void))
void i2cSlaveInit_R(uint32_t slave_addr )

{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
    GPIO_PORTB_AFSEL_R |= (GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTB_ODR_R |= 1<<3;
    GPIO_PORTB_PCTL_R |= ((3<<8) | 3<<12);

    I2C0_MCR_R |= 1<<5; // ACT AS Slave
    I2C0_SOAR_R = slave_addr<<1;
    I2C0_SCSR_R = 1;
    I2C0_SIMR_R = 1;

//    I2CIntRegister(I2C0_BASE, (*pfnHandler));
//    IntRegister(INT_I2C0,  (*pfnHandler));
    I2CIntRegister(I2C0_BASE, (*I2C0SlaveIntHandler));
    IntRegister(INT_I2C0,  (*I2C0SlaveIntHandler));
    IntEnable(INT_I2C0);

}


void i2cMasterSend_R(uint32_t slave_addr, char c[])
{
    int i;
    I2C0_MSA_R = (slave_addr <<1); // SLAVE ADDRESS WRITTEN IN MASTER
    I2C0_MSA_R &= ~(1); // send

    for (i=0;i<strlen(c);i++)
    {
        I2C0_MDR_R = c[i];
        I2C0_MCS_R = Start|Run;
        while (I2C0_MCS_R & Busy){}
    }
    I2C0_MCS_R = Stop;
    while (I2C0_MCS_R & BusBusy){}
    UARTprintf("data sent");

}


#endif /* I2C_TM4C123_H_ */
