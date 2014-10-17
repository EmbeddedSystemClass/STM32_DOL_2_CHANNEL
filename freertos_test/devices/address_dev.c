#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

uint8_t Address_Dev_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd =  GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    //GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd =  GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIOC->BSRRH=GPIO_Pin_0|GPIO_Pin_2;

    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1))
    {
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))
        {
        	return 0x5;
        }
        else
        {
        	return 0x7;
        }
    }
    else
    {
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))
        {
        	return 0x6;
        }
        else
        {
        	return 0x8;
        }
    }
    return 0x5;
}
