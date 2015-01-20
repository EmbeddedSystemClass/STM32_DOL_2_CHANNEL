#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include <misc.h>

//Инклуды от FreeRTOS:

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "hall_sensors.h"


#define HALL_SENSORS_PORT    		GPIOC
#define HALL_SENSORS_PORT_RCC 		RCC_AHB1Periph_GPIOC
#define HALL_SENSORS_PORT_EXTI 		EXTI_PortSourceGPIOC

#define HALL_SENSOR_0_PIN 			GPIO_Pin_0
#define HALL_SENSOR_1_PIN 			GPIO_Pin_1
#define HALL_SENSOR_2_PIN 			GPIO_Pin_2
#define HALL_SENSOR_3_PIN 			GPIO_Pin_3

#define HALL_SENSOR_0_PinSource 	GPIO_PinSource0
#define HALL_SENSOR_1_PinSource 	GPIO_PinSource1
#define HALL_SENSOR_2_PinSource 	GPIO_PinSource2
#define HALL_SENSOR_3_PinSource 	GPIO_PinSource3


void Hall_Sensors_Init(void)
{
	RCC_AHB1PeriphClockCmd(HALL_SENSORS_PORT_RCC, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = HALL_SENSOR_0_PIN|HALL_SENSOR_1_PIN|HALL_SENSOR_2_PIN|HALL_SENSOR_3_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(HALL_SENSORS_PORT, &GPIO_InitStructure);


	EXTI_InitTypeDef EXTI_InitStructure;

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_0_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_1_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_2_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_3_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);


	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
}

enum
{
	HALL_SENSOR_0=0,
	HALL_SENSOR_1,
	HALL_SENSOR_2,
	HALL_SENSOR_3,
};

static volatile uint8_t last_sensor=0xFF;

void EXTI0_IRQHandler(void)
{
		EXTI->PR = EXTI_Line0;
		switch(last_sensor)
		{
		 	 case HALL_SENSOR_0:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_1:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_3:
		 	 {

		 	 }
		 	 break;

		 	 default:
		 	 {

		 	 }
		}
		last_sensor=HALL_SENSOR_0;
}

void EXTI1_IRQHandler(void)
{
        EXTI->PR = EXTI_Line1;

		switch(last_sensor)
		{
		 	 case HALL_SENSOR_1:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_0:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_2:
		 	 {

		 	 }
		 	 break;

		 	 default:
		 	 {

		 	 }
		}
		last_sensor=HALL_SENSOR_1;
}

void EXTI2_IRQHandler(void)
{
        EXTI->PR = EXTI_Line2;

		switch(last_sensor)
		{
		 	 case HALL_SENSOR_2:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_1:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_3:
		 	 {

		 	 }
		 	 break;

		 	 default:
		 	 {

		 	 }
		}
		last_sensor=HALL_SENSOR_2;
}

void EXTI3_IRQHandler(void)
{
        EXTI->PR = EXTI_Line3;

		switch(last_sensor)
		{
		 	 case HALL_SENSOR_3:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_0:
		 	 {

		 	 }
		 	 break;

		 	 case HALL_SENSOR_2:
		 	 {

		 	 }
		 	 break;

		 	 default:
		 	 {

		 	 }
		}
		last_sensor=HALL_SENSOR_3;
}
