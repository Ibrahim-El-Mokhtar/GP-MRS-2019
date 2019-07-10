#ifndef DELAY_H_
#define DELAY_H_
void delayInit()
{
	NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 16000-1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |= 0x05;
}

void delay_ms(int t)
{
    int i=0;
    NVIC_ST_RELOAD_R = 16000-1;
    for (i=0 ; i<t ; i++)
    {
        NVIC_ST_RELOAD_R = 16000-1;
        NVIC_ST_CURRENT_R = 0;
       while ((NVIC_ST_CTRL_R & 0x00010000) == 0){

       }
    }
}

void delay_us(int t)
{
    int i=0;
    NVIC_ST_RELOAD_R = 16-1;
    for (i=0 ; i<t ; i++)
    {
        NVIC_ST_RELOAD_R = 16-1;
        NVIC_ST_CURRENT_R = 0;
       while ((NVIC_ST_CTRL_R & 0x00010000) == 0){

       }
    }
}
#endif
