#include "encoder.h"
#include "channels.h"
#include "watchdog.h"
extern struct Channel  channels[];//обобщенная структура каналов
extern struct task_watch task_watches[];

volatile uint32_t counter =0x80008000;
volatile uint32_t counter2=0x80008000;
volatile uint32_t period=0;
volatile uint32_t period_overload=0;

void DOL_Process( void *pvParameters );

void TIM3_IRQHandler(void)
{
 // if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	TIM3->SR = (uint16_t)~TIM_IT_Update;
    period=period_overload+TIM2->CNT;
    period_overload=0;
    TIM2->CNT=0;

    if(TIM3->CR1 & TIM_CR1_DIR)
    {
    	counter-=0x4;//0x1001 ;
    }
    else
    {
    	counter+=0x4;//0x1001;
    }
  }
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);



    if(TIM1->CR1 & TIM_CR1_DIR)
    {
    	counter2-=0x4;//0x1001;
    }
    else
    {
    	counter2+=0x4;//0x1001;
    }
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	  TIM2->SR = (uint16_t)~TIM_IT_Update;
	  period_overload+=10000;
  }
}

void delay(uint32_t time)
{
	while(time)
	{
		time--;
	}
}

void Freq_Measure_Init(void)
{
	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);
	  TIM_TimeBaseInitTypeDef timer_base;
	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 2400 - 1;
	  timer_base.TIM_Period = 10000;//500;
	  TIM_TimeBaseInit(TIM2, &timer_base);
	  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	  NVIC_EnableIRQ(TIM2_IRQn);
	  TIM_Cmd(TIM2, ENABLE);
}

void Encoder_Init(void)//инициализация таймера дола
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//тактируем портА
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);//тактируем таймер 3
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1EN, ENABLE);//тактируем таймер 1

	 	//настройка таймера дола
		TIM_TimeBaseInitTypeDef timer_base;
	    TIM_TimeBaseStructInit(&timer_base);
	    timer_base.TIM_Period = 0x3;//0x1000;
	    timer_base.TIM_Prescaler=0;
	    timer_base.TIM_ClockDivision= TIM_CKD_DIV1;
	    timer_base.TIM_CounterMode = TIM_CounterMode_Down | TIM_CounterMode_Up;
	    TIM_TimeBaseInit(TIM3, &timer_base);
	    TIM_TimeBaseInit(TIM1, &timer_base);

	    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	   // TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);


	    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	   // TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);


//	    TIM_ICInitTypeDef TIM_ICInitStruct;
//
//	    TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
//	    TIM_ICInitStruct.TIM_ICFilter=0x0;
//	    TIM_ICInit(TIM1, &TIM_ICInitStruct);
//	    TIM_ICInit(TIM3, &TIM_ICInitStruct);
//	    TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
//	    TIM_ICInitStruct.TIM_ICFilter=0x0;
//	    TIM_ICInit(TIM1, &TIM_ICInitStruct);
//	    TIM_ICInit(TIM3, &TIM_ICInitStruct);

	    //настройка прерывания дола
	    NVIC_InitTypeDef NVIC_InitStructure;

	    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);


	    //настройка пинов микроконтроллера
	    GPIO_InitTypeDef  GPIO_InitStructure;

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	   // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	    TIM1->CNT=0;
	    TIM3->CNT=0;

	    counter =0x80008000;
	    counter2=0x80008000;

	    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	  //  delay(1000);
	    TIM_Cmd(TIM1, ENABLE);
	  //  delay(1000);
	    TIM_Cmd(TIM3, ENABLE);
	    Freq_Measure_Init();
	    xTaskCreate(DOL_Process,(signed char*)"DOL_PROCESS",128, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void DOL_Process( void *pvParameters )//процесс обновления регистра дола-необязательный
{
	task_watches[DOL_TASK].task_status=TASK_IDLE;
	while(1)
	{
		task_watches[DOL_TASK].task_status=TASK_ACTIVE;


		channels[0].channel_data=counter+TIM3->CNT;//добавить критическую секцию
		channels[2].channel_data=counter+TIM1->CNT;

		if(period_overload>=20000)
		{
			channels[1].channel_data=0;
			period_overload=20000;
		}
		else
		{
			uint32_t freq=(10000<<8)/period;
			if(freq>=0xFFFF)
			{
				channels[1].channel_data=0xFFFF;
			}
			else
			{
				channels[1].channel_data=(10000<<8)/period;//counter2+TIM1->CNT;
			}
		}

		task_watches[DOL_TASK].counter++;
		vTaskDelay(50);
	}
}
