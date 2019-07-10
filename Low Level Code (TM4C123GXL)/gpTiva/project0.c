//*****************************************************************************
//
// project0.c - Example to demonstrate minimal TivaWare setup
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
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
#include "timer.h"


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

/* BOTS */
#define botNumber 1

int encoderTicksRight  , encoderTicksLeft;
int encoderFactorRight , encoderFactorLeft ;
int maxRPM_Right , maxRPM_Left;
double clockFreq = 62.5e-9;

    // rpm = (clock * (2 ^ VELDIV) * SPEED * 60) ÷ (LOAD * ppr * edges)
    //        16MHz           1       counts         3.2MHz * 490 * 4
    // rpm = encoderFactor * SPEED / encoderTicks
    // encoder factor calculation: (clockFreq) * 2 ^ 1 * 60 / (clockFreq/5) *  edges
    /*
        some notes :
        1. clock / load specifies how many times  you need the encoder to update (frequency of encoder readings per sec.)
        2. some of these encoders have 490 ticks and others have 240-250 ticks so i adjusted the encoderFactor   
    */  

#if botNumber == 1 
    encoderTicksRight = 490; 
    encoderTicksLeft  = 490;
    encoderFactorRight = 300;
    encoderFactorLeft = 150;
    maxRPM_Right = 139 ; 
    maxRPM_Left  = 141 ;
#elif  botNumber ==2
    encoderTicksRight = 490; 
    encoderTicksLeft  = 490;
    encoderFactorRight = 300;
    encoderFactorLeft = 150;
    maxRPM_Right = 120 ; 
    maxRPM_Left  = 125 ;
#elif  botNumber == 3
    encoderTicksRight = 490; 
    encoderTicksLeft  = 490;
    encoderFactorRight = 300;
    encoderFactorLeft = 300;
    maxRPM_Right = 120 ; 
    maxRPM_Left  = 127 ;
#elif botNumber == 4
    encoderTicksRight = 490; 
    encoderTicksLeft  = 490;
    encoderFactorRight = 300;
    encoderFactorLeft = 150;
    maxRPM_Right = 111 ; 
    maxRPM_Left  = 112 ;
#endif




#define slaveAddress 0x3C
// #define slaveAddressForIMU 0x52

#define encoderReload 1600000
void uartInit()
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){}
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200 ,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(0, 115200, 16000000);
}

int rpm_left =0,rpm_right=0;
uint8_t dataRx[5];
uint8_t datarx;
uint8_t dataToRPI[6];// = {'A','C','E','G'};

void
QEI0IntHandler(void){
    // rpm = (clock * (2 ^ VELDIV) * SPEED * 60) ÷ (LOAD * ppr * edges)
    //        16MHz           1       counts         3.2MHz * 490 * 4
        QEIIntClear(QEI0_BASE,QEI_INTTIMER);

        rpm_right = ((encoderFactorRight * QEIVelocityGet(QEI0_BASE))/encoderTicksRight );
//        UARTprintf("\t\t\trpm_right: %d", rpm_right);
        dataToRPI[0] = rpm_right;
        UARTprintf("\t\t\trpm_right: %d", rpm_right);


}

void
QEI1IntHandler(void){
    // rpm = (clock * (2 ^ VELDIV) * SPEED * 60) ÷ (LOAD * ppr * edges)
    //        16MHz           1       counts         3.2MHz * 490 * 4
        QEIIntClear(QEI1_BASE,QEI_INTTIMER);

        rpm_left = ((encoderFactorLeft * QEIVelocityGet(QEI1_BASE))/encoderTicksLeft);
//        UARTprintf("\t\t\t\trpm_left: %d \n", rpm_left);
        dataToRPI[1] = rpm_left;
        UARTprintf("\t\t\t\trpm_left: %d \n", rpm_left);


}




void
I2C0SlaveIntHandler(void)
{
    int i;
    int j;

    I2C0_SICR_R = 7 ;
    IntMasterDisable();



    uint8_t temp[3];

    for (i=0,j=0;(i<5) && (j<3);)
    {


        if (I2CSlaveStatus(I2C0_BASE) & (I2C_SLAVE_ACT_RREQ) )
        {

            temp[j] = (I2C0_SDR_R) & 0XFF ;
            I2CSlaveACKValueSet(I2C0_BASE, true);
            j++;
        }





        if (I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ)
        {

            I2C0_SDR_R = (dataToRPI[i]);
            i++;
        }



         if (j==3)
         {
             dataRx[0] = temp[0];
             dataRx[1] = temp[1];
             dataRx[2] = temp[2];
             j=0;
             break;
         }

         if (i >= 5)
         {
             i=0;
             break;
         }
         if (I2C0_SRIS_R & (1<<2))
         {

             I2C0_SICR_R = 7 ;

         }

    }

    IntMasterEnable();
    I2C0_SICR_R = 7 ;
//    delay_ms(1);
    delay_us(1);

}

void I2C3SlaveIntHandler(void)
{

}
//void interruptInit()
//{
//    //**********GPIO Interrupt
////    GPIOIntDisable(GPIO_PORTF_BASE,GPIO_PIN_0 );
//    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
//    GPIOIntRegister(GPIO_PORTF_BASE,(*motorRightHandler));
//    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0  , GPIO_RISING_EDGE);
//    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4 , GPIO_RISING_EDGE);
//    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
//    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);

//    IntRegister(INT_GPIOF,   (*motorRightHandler));
//    IntPrioritySet(INT_GPIOF, 0x00);
//    IntEnable(INT_GPIOF);
//
//}

//void gpioInit()
//{
////    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
////    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
////    //GPIO_PORTF_LOCK_R = 0x4C4F434B;
////    //**********GPIO
////    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4);
////    GPIO_PORTF_PUR_R = GPIO_PIN_0 | GPIO_PIN_4;
////    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE ,GPIO_PIN_1 | GPIO_PIN_2|GPIO_PIN_3 );
//}


void timerCap(uint32_t TIMER_BASE, uint32_t config)
{

        SYSCTL_RCGCTIMER_R |= 1<<2;
        while (SYSCTL_PRTIMER_R & (1<<2)==0){}
        SYSCTL_RCGCGPIO_R |= 1<<1;
        while (SYSCTL_PRGPIO_R & (1<<1)==0){}
        GPIO_PORTB_DIR_R &= ~(1);
        GPIO_PORTB_DEN_R |=1 | 1<<1;
        GPIO_PORTB_PDR_R |= 1 ;
        GPIO_PORTB_DR8R_R |= 3;
//        GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
        GPIO_PORTB_AFSEL_R |=1;
        GPIO_PORTB_PCTL_R &= ~0x0000000F;
        GPIO_PORTB_PCTL_R |= 0x00000007;

        TIMER2_CTL_R &= ~(1);
        TIMER2_CFG_R = 0x4;
        TIMER2_TAMR_R = 0x03 | 1<<2 ;
        TIMER2_CTL_R |= 0x0c;
        TIMER2_TAILR_R = 0xFFFF;

        TIMER2_ICR_R |= 1<<2;

}

void distance ()
{

        TIMER2_TAILR_R = 0xffff;
        TIMER2_CTL_R |= 1;
        TIMER2_ICR_R = 1<<2 | 1;

    //    TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);
        GPIO_PORTB_DATA_R &= ~(1<<1);
        delay_us(3);
        GPIO_PORTB_DATA_R |= 1<<1;
        delay_us(11);
        GPIO_PORTB_DATA_R &= ~(1<<1);

        while ((TIMER2_RIS_R & 1<<2) ==0)
        {
//            UARTprintf("FIRSTT %d\n",TimerValueGet(TIMER2_BASE, TIMER_A));
//            if (TIMER2_TAV_R & 0xFA00) // 4 ms
////            if ((TIMER2_RIS_R & 1))
//                {
//                UARTprintf("!!!!!!!!!!!!!!!!!!!!!!!!!! RIS 1 !!!!!");
//
//                TIMER2_ICR_R = 1;
//
//                TIMER2_CTL_R &= ~(1);
//                return 137;
//                break;
//                }
        }

        TIMER2_ICR_R = 1<<2 ;
        uint32_t time1 = TimerValueGet(TIMER2_BASE, TIMER_A);

        while (!(TIMER2_RIS_R & 1<<2))
        {
//            UARTprintf("#####################################################"
//                    "########################################################Second %x\n",TimerValueGet(TIMER2_BASE, TIMER_A));

//            if ((TIMER2_RIS_R & 1))
//            if (TIMER2_TAV_R & 0xFFFC) // 4 ms
//               {
//                UARTprintf("!!!!!!!!!!!!!!!!!!!!!!!!!! RIS 2 !!!!!");
//                TIMER2_ICR_R = 1;
//
//                TIMER2_CTL_R &= ~(1);
//                return 137;
//                break;
//               }
        }

        TIMER2_ICR_R = 1<<2 ;

        uint32_t time2 = TimerValueGet(TIMER2_BASE, TIMER_A);

        TIMER2_CTL_R &= ~(1);
        UARTprintf("timer 2 %d , timer 1 %d",time2,time1);

        int dist = (int)(((time1- time2)/928)); //*62.5e-9 * 5882));

        delay_us(400);
        if (dist>150){
            dataToRPI[2]=150 & 0xff;
        }
        else
        {
            dataToRPI[2]=dist & 0xff;
        }
        UARTprintf("DISTANCE :%d \n",dataToRPI[2] );
}



int
main(void)
{
//    ROM_FPULazyStackingEnable();
    uartInit();
    delayInit();
//    //
//    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
//    //
//    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
//                    SYSCTL_OSC_MAIN);



    pwmInit(PWM0_BASE, PWM_GEN_2);
//    pwmMotorRight(1, 19999);
    QEIInit(QEI0_BASE, encoderReload, 240);
//    delay_us(100);
    pwmInit(PWM0_BASE, PWM_GEN_1);
//    pwmMotorLeft(1, 19999);
    QEIInit(QEI1_BASE, encoderReload, 240);



    timerInit(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_ONE_SHOT);//CONFIGURE TIMER INIT
//    timerCap(TIMER2_BASE, 2);
//    encoder();
//    pwmInit();
    i2cSlaveInit(I2C0_BASE,slaveAddress);

//    delay_ms(200);


    // char string[] = "the quick brown fox jumps over the lazy dog";
    IntMasterEnable();
//    IntMasterDisable();


//    float Kp_R = 0.24, Ki_R = 10.1, Kd_R = 0,
//            Kp_L = 0.24, Ki_L =10.1, Kd_L =0;
    double f = 2.4;
    double Ku = 3, Pu = 0.012*2*f;

    // PI parameters:
   double Kp_R = 0.45 * Ku, Ki_R = 0.54 * (Ku/Pu), Kd_R = 0;
//   double Kp_R = 0.5*Ku, Ki_R = 0, Kd_R = 0;

    // PID parameters:
//    double Kp_R = 0.6 * Ku, Ki_R = 1.2 * (Ku/Pu), Kd_R = 0.075 * Ku * Pu; //Figure 5 After Tuning             // 1, 2, 1
    // double Kp_R = Ku/5, Ki_R = 0.4 * (Ku/Pu), Kd_R =  Ku * Pu /15; //Figure 5 After Tuning             // 1, 2, 1


    int error_R=0,error_L = 0;
    int prev_error_R = 0,prev_error_L = 0;

    int Int_prev_R = 0;
    int Int_prev_L = 0;
    double A=0,B=0,C = Ki_R;


    int target_rpm_R = (int)dataRx[0];
    int target_rpm_L = (int)dataRx[1];
    double PIDValue_R=0 ,PIDValue_L = 0,
            PIDValue_1_R=0,PIDValue_1_L=0,
            PIDValue_2_R=0,PIDValue_2_L=0;
    double sampleTime=0.013;

    double numberOfTicksPercentage_R =0 , numberOfTicksPercentage_L =0;
    int numberOfTicks_R =0,numberOfTicks_L=0;
//    UARTprintf("%d", (int)(Kp_L *100)/10);

//    NTS = N*sampleTime;
//    A0_R = 1 + NTS;
//    A1_R = -2 - NTS;
//    A2_R = 1;
//    B0_R = Kp_R * (1+NTS) + Ki_R * sampleTime * (1 + NTS) + Kd_R * N;
//    B1_R = -Kp_R *(2 + NTS) - Ki_R *sampleTime - 2 * Kd_R * N;
//    B2_R = Kp_R + Kd_R * N;
//    ku1_R = A1_R / A0_R; ku2_R = A2_R / A0_R;
//    ke0_R = B0_R / A0_R; ke1_R = B1_R / A0_R; ke2_R = B2_R;
    delay_us(30);


    while(1)
    {
//        UARTprintf("******in main \n");
//        delay_ms(80);
        target_rpm_R = (int)(dataRx[0]);
        target_rpm_L = (int)(dataRx[1]);
//        distance();
//        target_rpm_R=100;
//        target_rpm_L=100;
//        dataRx[2] = 4;
        delay_us(10);

//             Enable the timers.
            //
//            TimerLoadSet(WTIMER0_BASE, TIMER_A, 16000000);
            TimerEnable(WTIMER0_BASE, TIMER_A);

//*********************************************************************######################################




            
//            UARTprintf("sha8al fel MAIN *__* \n");
            //################WORKING#####################
            error_R = target_rpm_R - rpm_right;
            error_L = target_rpm_L - rpm_left;
//            UARTprintf("kp= %d \t" ,(int)(Kp_R*100)); delay_us(1500);

           A = Kp_R + (Ki_R*sampleTime)/2 + (float)(Kd_R/sampleTime);              delay_ms(10);
//            UARTprintf("A = %d \t" ,(int)(A*100)); delay_us(150);

           B = (Ki_R*sampleTime)/(float)2 - (Kd_R/sampleTime);                    delay_us(15);
//            UARTprintf("B= %d \t" ,B); delay_us(15);

           PIDValue_R = A*error_R + B*prev_error_R + C*Int_prev_R; delay_us(15);
           PIDValue_L = A*error_L + B*prev_error_L + C*Int_prev_L; delay_us(15);

        // TEST
            // numberOfTicksPercentage_R = (double)(rpm_right+error_R) / maxRPM_Right;delay_us(15); //139 being 100% rpm according to mapping excel sheet
            // numberOfTicksPercentage_L = (double)(rpm_left+error_L)/maxRPM_Left; delay_us(15);
        // TEST END 
           numberOfTicksPercentage_R = (double)(PIDValue_R) / maxRPM_Right;delay_us(15); //139 being 100% rpm according to mapping excel sheet
           numberOfTicksPercentage_L = (double)(PIDValue_L) /maxRPM_Left; delay_us(15);
//            UARTprintf("\t\t\t\t %d ", (int)(numberOfTicksPercentage_R*100));
//
            if(numberOfTicksPercentage_R>1)numberOfTicksPercentage_R=1;
            if(numberOfTicksPercentage_L>1)numberOfTicksPercentage_L=1;

            numberOfTicks_R = (int)((1 - numberOfTicksPercentage_R) * pwmFrequency); delay_us(15);
            numberOfTicks_L = (int)((1 - numberOfTicksPercentage_L) * pwmFrequency); delay_us(15);
//            UARTprintf("\t\t\t\t %d \t ", (int)(numberOfTicks_R));

            if (numberOfTicks_R > pwmFrequency-1 | numberOfTicks_R < 0) numberOfTicks_R = pwmFrequency-1;
            else if (numberOfTicks_R == 0) numberOfTicks_R = 1;

            if (numberOfTicks_L > pwmFrequency-1 | numberOfTicks_L < 0 ) numberOfTicks_L = pwmFrequency-1;
            else if (numberOfTicks_L == 0) numberOfTicks_L = 1;


            delay_us(10);


            switch (dataRx[2])
            {
                case 0:
                    break;
                case 2:
                    pwmMotorRight(numberOfTicks_R, pwmFrequency - 1);
                    pwmMotorLeft(pwmFrequency -1,numberOfTicks_L);
                    break;
                case 1:
                    pwmMotorRight(pwmFrequency - 1,numberOfTicks_R);
                    pwmMotorLeft(numberOfTicks_L,pwmFrequency -1);
                    break;
                case 3:
                    pwmMotorRight(numberOfTicks_R, pwmFrequency - 1);
                    pwmMotorLeft(numberOfTicks_L, pwmFrequency -1);
                    break;
                case 4:
                    pwmMotorRight(pwmFrequency - 1,numberOfTicks_R);
                    pwmMotorLeft(pwmFrequency -1,numberOfTicks_L);
                    break;
                default:
                    break;
            }


            Int_prev_R = Int_prev_R + ((error_R - prev_error_R) * (sampleTime/2));delay_us(15);
            Int_prev_L = Int_prev_L + ((error_L - prev_error_L) * (sampleTime/2));delay_us(15);

//            delay_ms(1);

            prev_error_R = error_R;
            prev_error_L = error_L;

            sampleTime = 1000*(1-(TimerValueGet(WTIMER0_BASE, TIMER_A) /(double)16000000));
//            sampleTime = 0.040;
//            delay_us(10);

//            UARTprintf("\t\t\t\t\t\t\t\t\t\t sampling time %d ", (int)(sampleTime*1000));

      //  UARTprintf("clock ticks %" PRIu32 "\n",TimerValueGet(WTIMER0_BASE, TIMER_A));


        if (WTIMER0_RIS_R & 1)
        {
            WTIMER0_ICR_R = 1;
            UARTprintf("WWIIIIHHHHHHIIIIIIIIIIIIIIIIIIIIIIIII\n");
        }
        TimerLoadSet(WTIMER0_BASE, TIMER_A, 16000000);

            TimerDisable(WTIMER0_BASE, TIMER_A);
//            UARTprintf("WWIIIIHHHHHHIIIIIIIIIIIIIIIIIIIIIIIII\n");

        //delay
    }
}
