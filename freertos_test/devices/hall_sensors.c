#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include <misc.h>


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
volatile uint32_t period_overload=0;

#define MEASURE_TIM_PERIOD		10000
#define ONE_SECOND_TIMEPERIOD	10000
#define	MAX_PERIOD				40

#define PERIOD_QUEUE_LENGTH	8
struct period_queue
{
	uint32_t period[PERIOD_QUEUE_LENGTH];
	uint8_t	 counter;
};

volatile struct period_queue p_queue;

void Freq_Measure_Init(void);


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

	GPIO_InitStructure.GPIO_Pin = ENC_0_PIN|ENC_1_PIN|GPIO_Pin_10|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ENC_PORT, &GPIO_InitStructure);

	EXTI_InitTypeDef EXTI_InitStructure;

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_0_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_1_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_2_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(HALL_SENSORS_PORT_EXTI, HALL_SENSOR_3_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);


	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel =  EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	Freq_Measure_Init();

	ENC_PORT->BSRRH=GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_8;

	NVIC_EnableIRQ( EXTI9_5_IRQn);
//	NVIC_EnableIRQ(EXTI1_IRQn);
//	NVIC_EnableIRQ(EXTI2_IRQn);
//	NVIC_EnableIRQ(EXTI3_IRQn);

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


void TIM2_IRQHandler(void)
{
	  TIM2->SR = (uint16_t)~TIM_IT_Update;
	  period_overload+=MEASURE_TIM_PERIOD;
}


void Freq_Measure_Init(void)
{
	  TIM_TimeBaseInitTypeDef timer_base;

	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);

	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 240 - 1;
	  timer_base.TIM_Period = MEASURE_TIM_PERIOD;
	  TIM_TimeBaseInit(TIM2, &timer_base);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	  NVIC_EnableIRQ(TIM2_IRQn);
	  TIM_Cmd(TIM2, ENABLE);

}


void EXTI9_5_IRQHandler(void)
{
  if((EXTI->PR & EXTI_Line6) != (uint32_t)RESET)
  {
	  EXTI->PR = EXTI_Line6;

    sensors_state=((uint8_t)((HALL_SENSORS_PORT->IDR>>6)&IDR_MASK))&0x3;
    		switch(sensors_state)
    		{
    		 	 case HALL_STATE_0:
    		 	 {
    		 		 counter--;

    		 	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
    		 	    p_queue.counter++;
    		 	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);

    		 	    period_overload=0;

    		 	    TIM2->CNT=0;

    		 	   // ENC_PORT->ODR^=GPIO_Pin_10;
    		 	 }
    		 	 break;

    		 	 case HALL_STATE_1:
    		 	 {
    		 		 counter++;

    		 	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
    		 	    p_queue.counter++;
    		 	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);

    		 	    period_overload=0;

    		 	    TIM2->CNT=0;

    		 	    //ENC_PORT->ODR^=GPIO_Pin_10;
    		 	 }
    		 	 break;

    		 	 case HALL_STATE_2:
    		 	 {
    		 		 counter--;
    		 		//ENC_PORT->ODR^=GPIO_Pin_10;
    		 	 }
    		 	 break;

    		 	 case HALL_STATE_3:
    		 	 {
    		 		counter++;
    		 		//ENC_PORT->ODR^=GPIO_Pin_10;
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

  if((EXTI->PR & EXTI_Line7) != (uint32_t)RESET)
  {
	  EXTI->PR = EXTI_Line7;


	sensors_state=(((uint8_t)((HALL_SENSORS_PORT->IDR>>6)&IDR_MASK))>>1)&0x3;
	switch(sensors_state)
	{
	 	 case HALL_STATE_0:
	 	 {
	 		counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_8;
	 	 }
	 	 break;

	 	 case HALL_STATE_1:
	 	 {
	 		 counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_8;
	 	 }
	 	 break;

	 	 case HALL_STATE_2:
	 	 {
	 		 counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_8;
	 	 }
	 	 break;

	 	 case HALL_STATE_3:
	 	 {
	 		counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_8;
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

  if((EXTI->PR & EXTI_Line8) != (uint32_t)RESET)
  {
	  EXTI->PR = EXTI_Line8;


	sensors_state=(((uint8_t)((HALL_SENSORS_PORT->IDR>>6)&IDR_MASK))>>2)&0x3;
	switch(sensors_state)
	{
	 	 case HALL_STATE_0:
	 	 {
	 		counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_9;
	 	 }
	 	 break;

	 	 case HALL_STATE_1:
	 	 {
	 		 counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_9;
	 	 }
	 	 break;

	 	 case HALL_STATE_2:
	 	 {
	 		 counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_9;
	 	 }
	 	 break;

	 	 case HALL_STATE_3:
	 	 {
	 		counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_9;
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

  if((EXTI->PR & EXTI_Line9) != (uint32_t)RESET)
  {
	  EXTI->PR = EXTI_Line9;


    sensors_state=(((uint8_t)((HALL_SENSORS_PORT->IDR>>6)&IDR_MASK))>>2)&0x3;

	switch(sensors_state)
	{
	 	 case HALL_STATE_1:
	 	 {
	 		counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_10;
	 	 }
	 	 break;

	 	 case HALL_STATE_2:
	 	 {
	 		 counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_10;
	 	 }
	 	 break;

	 	 case HALL_STATE_3:
	 	 {
	 		 counter--;
	 		ENC_PORT->ODR^=GPIO_Pin_10;
	 	 }
	 	 break;

	 	 case HALL_STATE_0:
	 	 {
	 		counter++;
	 		ENC_PORT->ODR^=GPIO_Pin_10;
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

}


//void EXTI0_IRQHandler(void)
//{
//		EXTI->PR = EXTI_Line0;
//
////	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
////	    p_queue.counter++;
////	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
////
////	    period_overload=0;
////
////	    TIM2->CNT=0;
//
//		sensors_state=((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))&0x3;
//		switch(sensors_state)
//		{
//		 	 case HALL_STATE_0:
//		 	 {
//		 		 counter--;
//
//		 	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
//		 	    p_queue.counter++;
//		 	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
//
//		 	    period_overload=0;
//
//		 	    TIM2->CNT=0;
//
//		 	    ENC_PORT->ODR^=GPIO_Pin_10;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_1:
//		 	 {
//		 		 counter++;
//
//		 	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
//		 	    p_queue.counter++;
//		 	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
//
//		 	    period_overload=0;
//
//		 	    TIM2->CNT=0;
//
//		 	    ENC_PORT->ODR^=GPIO_Pin_10;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_2:
//		 	 {
//		 		 counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_3:
//		 	 {
//		 		counter++;
//		 	 }
//		 	 break;
//		}
//
//		switch(counter&ENC_MASK)
//		{
//			case 0x0:
//			{
//				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
//			}
//			break;
//
//			case 0x1:
//			{
//				ENC_PORT->BSRRH=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x2:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x3:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRH=ENC_1_PIN;
//			}
//			break;
//		}
//}

//void EXTI1_IRQHandler(void)
//{
//        EXTI->PR = EXTI_Line1;
//
////	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
////	    p_queue.counter++;
////	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
////
////	    period_overload=0;
////
////	    TIM2->CNT=0;
//
//		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>1)&0x3;
//		switch(sensors_state)
//		{
//		 	 case HALL_STATE_0:
//		 	 {
//		 		counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_1:
//		 	 {
//		 		 counter++;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_2:
//		 	 {
//		 		 counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_3:
//		 	 {
//		 		counter++;
//		 	 }
//		 	 break;
//		}
//
//		switch(counter&ENC_MASK)
//		{
//			case 0x0:
//			{
//				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
//			}
//			break;
//
//			case 0x1:
//			{
//				ENC_PORT->BSRRH=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x2:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x3:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRH=ENC_1_PIN;
//			}
//			break;
//		}
//}

//void EXTI2_IRQHandler(void)
//{
//        EXTI->PR = EXTI_Line2;
//
////	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
////	    p_queue.counter++;
////	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
////
////	    period_overload=0;
////
////	    TIM2->CNT=0;
//
//		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>2)&0x3;
//		switch(sensors_state)
//		{
//		 	 case HALL_STATE_0:
//		 	 {
//		 		counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_1:
//		 	 {
//		 		 counter++;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_2:
//		 	 {
//		 		 counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_3:
//		 	 {
//		 		counter++;
//		 	 }
//		 	 break;
//		}
//
//		switch(counter&ENC_MASK)
//		{
//			case 0x0:
//			{
//				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
//			}
//			break;
//
//			case 0x1:
//			{
//				ENC_PORT->BSRRH=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x2:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x3:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRH=ENC_1_PIN;
//			}
//			break;
//		}
//}

//void EXTI3_IRQHandler(void)
//{
//        EXTI->PR = EXTI_Line3;
//
////	    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
////	    p_queue.counter++;
////	    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);
////
////	    period_overload=0;
////
////	    TIM2->CNT=0;
//
//		sensors_state=(((uint8_t)(HALL_SENSORS_PORT->IDR&IDR_MASK))>>2)&0x3;
//		switch(sensors_state)
//		{
//		 	 case HALL_STATE_1:
//		 	 {
//		 		counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_2:
//		 	 {
//		 		 counter++;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_3:
//		 	 {
//		 		 counter--;
//		 	 }
//		 	 break;
//
//		 	 case HALL_STATE_0:
//		 	 {
//		 		counter++;
//		 	 }
//		 	 break;
//		}
//
//		switch(counter&ENC_MASK)
//		{
//			case 0x0:
//			{
//				ENC_PORT->BSRRH=(ENC_0_PIN|ENC_1_PIN);
//			}
//			break;
//
//			case 0x1:
//			{
//				ENC_PORT->BSRRH=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x2:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRL=ENC_1_PIN;
//			}
//			break;
//
//			case 0x3:
//			{
//				ENC_PORT->BSRRL=ENC_0_PIN;
//				ENC_PORT->BSRRH=ENC_1_PIN;
//			}
//			break;
//		}
//}

//#define SWAP(A, B) { uint32_t t = A; A = B; B = t; }
//
//void bubblesort(uint32_t *a, uint32_t n)
//{
//  uint16_t i, j;
//
//  for (i = n - 1; i > 0; i--)
//  {
//    for (j = 0; j < i; j++)
//    {
//      if (a[j] > a[j + 1])
//        SWAP( a[j], a[j + 1] );
//    }
//  }
//}

void Hall_Process( void *pvParameters )//
{
//	task_watches[DOL_TASK].task_status=TASK_IDLE;
	static uint8_t period_overload_flag=0x1;
	uint8_t i=0;

	while(1)
	{
//		task_watches[DOL_TASK].task_status=TASK_ACTIVE;

		channels[0].channel_data=counter;//

		if(period_overload>=(MEASURE_TIM_PERIOD*MAX_PERIOD))
		{
					channels[1].channel_data=0;
					channels[2].channel_data=0;
					period_overload=MEASURE_TIM_PERIOD*MAX_PERIOD;
					period_overload_flag=0x1;


					for(i=0;i<PERIOD_QUEUE_LENGTH;i++)
				    {
				    	p_queue.period[i]=MEASURE_TIM_PERIOD*MAX_PERIOD;
				    }
					p_queue.counter=0;
		}
		else
		{
				    uint32_t sum_period=0;

				    if(period_overload_flag)
				    {
				    	if(p_queue.period[1]==MEASURE_TIM_PERIOD*MAX_PERIOD)
				    	{
				    		continue;
				    	}

						for(i=0;i<PERIOD_QUEUE_LENGTH;i++)
					    {
					    	p_queue.period[i]=p_queue.period[(p_queue.counter-1)&(PERIOD_QUEUE_LENGTH-1)];
					    }
				    	period_overload_flag=0x0;
				    }

					for(i=0;i<PERIOD_QUEUE_LENGTH;i++)
				    {
				    	sum_period+=p_queue.period[i];
				    }

//				    uint32_t temp_mas[PERIOD_QUEUE_LENGTH];
//
//					for(i=0;i<PERIOD_QUEUE_LENGTH;i++)
//					{
//						temp_mas[i]=p_queue.period[i];
//					}
					//bubblesort(temp_mas,PERIOD_QUEUE_LENGTH);


					uint32_t freq_div_10=0;
					uint32_t freq=(((MEASURE_TIM_PERIOD*10)<<8)*PERIOD_QUEUE_LENGTH)/sum_period;
					//uint32_t freq=(((MEASURE_TIM_PERIOD*10)<<8))/temp_mas[PERIOD_QUEUE_LENGTH>>1];

					if(freq%10<5)
					{
						 freq_div_10=freq/10;
					}
					else
					{
						 freq_div_10=(freq/10)+1;
					}

					if(freq>=0xFFFF)
					{
						channels[1].channel_data=0xFFFF;
					}
					else
					{
						channels[1].channel_data=freq;//((MEASURE_TIM_PERIOD)<<8)/period;
					}


					//uint32_t freq_div_10=freq/10;//(((MEASURE_TIM_PERIOD*10)<<8)*PERIOD_QUEUE_LENGTH)/sum_period;
					if(freq_div_10>=0xFFFF)
					{
						channels[2].channel_data=0xFFFF;
					}
					else
					{
						channels[2].channel_data=freq_div_10;//((MEASURE_TIM_PERIOD)<<8)/period;
					}
		}

//		task_watches[DOL_TASK].counter++;
		vTaskDelay(200);
	}
}
