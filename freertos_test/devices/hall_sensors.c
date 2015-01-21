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

#include "channels.h"
#include "watchdog.h"
extern struct Channel  channels[];

static volatile uint8_t sensors_state=0;

static volatile uint32_t counter=0x80008000;


void Hall_Process( void *pvParameters );

void Hall_Sensors_Init(void)
{
	RCC_AHB1PeriphClockCmd(HALL_SENSORS_PORT_RCC|ENC_PORT_RCC, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = HALL_SENSOR_0_PIN|HALL_SENSOR_1_PIN|HALL_SENSOR_2_PIN|HALL_SENSOR_3_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(HALL_SENSORS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ENC_0_PIN|ENC_1_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ENC_PORT, &GPIO_InitStructure);

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);

	switch(counter&ENC_MASK)
	{
		case 0x0:
		{
			ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
		}
		break;

		case 0x1:
		{
			ENC_PORT->BSRRH=ENC_0_PIN;
			ENC_PORT->BSRRL=ENC_1_PIN;
		}
		break;

		case 0x2:
		{
			ENC_PORT->BSRRL=ENC_0_PIN;
			ENC_PORT->BSRRL=ENC_1_PIN;
		}
		break;

		case 0x3:
		{
			ENC_PORT->BSRRL=ENC_0_PIN;
			ENC_PORT->BSRRH=ENC_1_PIN;
		}
		break;
	}

	xTaskCreate(Hall_Process,(signed char*)"HALL_PROCESS",128, NULL, tskIDLE_PRIORITY + 1, NULL);
}






void EXTI0_IRQHandler(void)
{
		EXTI->PR = EXTI_Line0;

		sensors_state=((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))&0x3;
		switch(sensors_state)
		{
		 	 case HALL_STATE_0:
		 	 {
		 		counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_1:
		 	 {
		 		 counter++;
		 	 }
		 	 break;

		 	 case HALL_STATE_2:
		 	 {
		 		 counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_3:
		 	 {
		 		counter++;
		 	 }
		 	 break;
		}

		switch(counter&ENC_MASK)
		{
			case 0x0:
			{
				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
			}
			break;

			case 0x1:
			{
				ENC_PORT->BSRRH=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x2:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x3:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRH=ENC_1_PIN;
			}
			break;
		}
}

void EXTI1_IRQHandler(void)
{
        EXTI->PR = EXTI_Line1;

		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>1)&0x3;
		switch(sensors_state)
		{
		 	 case HALL_STATE_0:
		 	 {
		 		counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_1:
		 	 {
		 		 counter++;
		 	 }
		 	 break;

		 	 case HALL_STATE_2:
		 	 {
		 		 counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_3:
		 	 {
		 		counter++;
		 	 }
		 	 break;
		}

		switch(counter&ENC_MASK)
		{
			case 0x0:
			{
				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
			}
			break;

			case 0x1:
			{
				ENC_PORT->BSRRH=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x2:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x3:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRH=ENC_1_PIN;
			}
			break;
		}
}

void EXTI2_IRQHandler(void)
{
        EXTI->PR = EXTI_Line2;

		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>2)&0x3;
		switch(sensors_state)
		{
		 	 case HALL_STATE_0:
		 	 {
		 		counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_1:
		 	 {
		 		 counter++;
		 	 }
		 	 break;

		 	 case HALL_STATE_2:
		 	 {
		 		 counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_3:
		 	 {
		 		counter++;
		 	 }
		 	 break;
		}

		switch(counter&ENC_MASK)
		{
			case 0x0:
			{
				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
			}
			break;

			case 0x1:
			{
				ENC_PORT->BSRRH=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x2:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x3:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRH=ENC_1_PIN;
			}
			break;
		}
}

void EXTI3_IRQHandler(void)
{
        EXTI->PR = EXTI_Line3;

		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>2)&0x3;
		switch(sensors_state)
		{
		 	 case HALL_STATE_1:
		 	 {
		 		counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_2:
		 	 {
		 		 counter++;
		 	 }
		 	 break;

		 	 case HALL_STATE_3:
		 	 {
		 		 counter--;
		 	 }
		 	 break;

		 	 case HALL_STATE_0:
		 	 {
		 		counter++;
		 	 }
		 	 break;
		}

		switch(counter&ENC_MASK)
		{
			case 0x0:
			{
				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
			}
			break;

			case 0x1:
			{
				ENC_PORT->BSRRH=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x2:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRL=ENC_1_PIN;
			}
			break;

			case 0x3:
			{
				ENC_PORT->BSRRL=ENC_0_PIN;
				ENC_PORT->BSRRH=ENC_1_PIN;
			}
			break;
		}
}

void Hall_Process( void *pvParameters )//
{
//	task_watches[DOL_TASK].task_status=TASK_IDLE;

	while(1)
	{
//		task_watches[DOL_TASK].task_status=TASK_ACTIVE;

		channels[0].channel_data=counter;//

//		task_watches[DOL_TASK].counter++;
		vTaskDelay(50);
	}
}
