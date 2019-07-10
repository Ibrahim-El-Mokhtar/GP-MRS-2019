#ifndef I2C_TIVAWARE_H_
#define I2C_TIVAWARE_H_
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
extern void I2C3SlaveIntHandler(void);

extern uint8_t dataToRPI[6];

void i2cMasterInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
    GPIOPinConfigure(GPIO_PB2_I2C0SCL); //PB2
    GPIOPinConfigure(GPIO_PB3_I2C0SDA); //PB3
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
//    GPIO_PORTB_ODR_R = 1<<3;
//    GPIO_PORTB_PUR_R = (1<<2) | (1<<3);
    //master init clock
    I2CMasterInitExpClk(I2C0_BASE, 16000000, false);
//    I2CMasterGlitchFilterConfigSet(I2C0_BASE, I2C_MASTER_GLITCH_FILTER_8);
//    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddress, true);
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

//    //Master Send
//    I2CMasterDataPut(I2C0_BASE, ’Q’);
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
//    while(I2CMasterBusBusy(I2C0_BASE)){}
}

void i2cSlaveInit(uint32_t i2cBase,uint32_t slave_addr)
{
    if (i2cBase == I2C0_BASE )
    {
         SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
    GPIOPinConfigure(GPIO_PB2_I2C0SCL); //PB2
    GPIOPinConfigure(GPIO_PB3_I2C0SDA); //PB3

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIO_PORTB_PUR_R = (1<<2) | (1<<3);
    //master init clock
    I2CSlaveEnable(i2cBase);
    I2CSlaveInit(i2cBase, slave_addr);
    I2CSlaveACKOverride(i2cBase, true);
//    I2CSlaveACKValueSet(I2C0_BASE, true);
//    I2C0_SACKCTL_R = 1;
//    while(!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ)){UARTprintf("line 340 - master\t");}
//    I2C0_PC_R = 1;
    I2CIntRegister(I2C0_BASE, (*I2C0SlaveIntHandler));
//    I2QEIIntRegisterCSlaveIntEnable(I2C0_BASE);
    I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);

    IntPrioritySet(INT_I2C0, 0x00);
//    IntRegister(INT_I2C0,  (*I2C0SlaveIntHandler));
    IntEnable(INT_I2C0);

    }
    else if (i2cBase == I2C3_BASE)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
       while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3)){}
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
       while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){}
       GPIOPinConfigure(GPIO_PD0_I2C3SCL); //PB2
       GPIOPinConfigure(GPIO_PD1_I2C3SDA); //PB3

       GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
       GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
       GPIO_PORTD_PUR_R = (1) | (1<<1);

       //master init clock
       I2CSlaveEnable(i2cBase);
       I2CSlaveInit(i2cBase, slave_addr);
       I2CSlaveACKOverride(i2cBase, true);
   //    I2CSlaveACKValueSet(I2C0_BASE, true);
   //    I2C0_SACKCTL_R = 1;
   //    while(!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ)){UARTprintf("line 340 - master\t");}

       I2CIntRegister(I2C3_BASE, (*I2C3SlaveIntHandler));
   //    I2QEIIntRegisterCSlaveIntEnable(I2C0_BASE);
       I2CSlaveIntEnableEx(I2C3_BASE, I2C_SLAVE_INT_DATA);

       IntPrioritySet(INT_I2C3, 0x00);
   //    IntRegister(INT_I2C0,  (*I2C0SlaveIntHandler));
       IntEnable(INT_I2C3);

    }
}

void I2CSendString(uint32_t slave_addr, char array[])
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, array[0]);

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(array[1] == '\0')
    {
        //Initiate send of data from the MCU
        while(I2CMasterBusBusy(I2C0_BASE));
        while(I2CMasterBusy(I2C0_BASE)){
            UARTprintf("value of a = 0x%08x \n",I2C0_MCS_R);
            UARTprintf("5ARA stuck \n");
        }
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        I2C0_MCS_R |= 1<<3;

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE)){
            UARTprintf("value of a = 0x%08x \n",I2C0_MCS_R);
            UARTprintf("stuck");
        }
//        while (I2C0_MCS_R & Busy){UARTprintf("stuck");}
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        while(I2CMasterBusBusy(I2C0_BASE));

        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        I2C0_MCS_R |= 1<<3;

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));


        //initialize index into array
        uint8_t i = 1;

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        while(array[i + 1] != '\0')
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, array[i++]);

            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
//            UARTprintf("value of a = 0x%08x \n",I2C0_MCS_R);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, array[i]);

        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

    }
    UARTprintf("value of a = 0x%08x \n",I2C0_MCS_R);

}




#endif
