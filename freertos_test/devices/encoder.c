#include "encoder.h"
#include "channels.h"
#include "watchdog.h"
extern struct Channel  channels[];
extern struct task_watch task_watches[];

#define MEASURE_TIM_PERIOD		10000
#define ONE_SECOND_TIMEPERIOD	10000
#define	MAX_PERIOD				40

volatile uint32_t counter =0x80008000;
volatile uint32_t fast_freq_counter=0;
volatile uint32_t period_overload=0;


#define PERIOD_QUEUE_LENGTH	8
struct period_queue
{
	uint32_t period[PERIOD_QUEUE_LENGTH];
	uint8_t	 counter;
};

volatile struct period_queue p_queue;


void DOL_Process( void *pvParameters );

void TIM3_IRQHandler(void)
{
	TIM3->SR = (uint16_t)~TIM_IT_Update;
    p_queue.period[p_queue.counter]=period_overload+TIM2->CNT;
    p_queue.counter++;
    p_queue.counter&=(PERIOD_QUEUE_LENGTH-1);

    period_overload=0;


    TIM2->CNT=0;

    if(TIM3->CR1 & TIM_CR1_DIR)
    {
    	counter-=ENC_INT_PERIOD;// ;
    }
    else
    {
    	counter+=ENC_INT_PERIOD;//;
    }
}

void TIM2_IRQHandler(void)
{
 // if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
 // {
	  TIM2->SR = (uint16_t)~TIM_IT_Update;
	  period_overload+=MEASURE_TIM_PERIOD;
 // }
}

void TIM4_IRQHandler(void)
{
	TIM4->SR = (uint16_t)~TIM_IT_Update;
	channels[2].channel_data=fast_freq_counter;
	fast_freq_counter=0;
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

void Encoder_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

		TIM_TimeBaseInitTypeDef timer_base;
	    TIM_TimeBaseStructInit(&timer_base);
	    timer_base.TIM_Period = ENC_INT_PERIOD-1;//;
	    timer_base.TIM_Prescaler=0;
	    timer_base.TIM_ClockDivision= TIM_CKD_DIV1;
	    timer_base.TIM_CounterMode = TIM_CounterMode_Down | TIM_CounterMode_Up;
	    TIM_TimeBaseInit(TIM3, &timer_base);
	    TIM_TimeBaseInit(TIM1, &timer_base);

	    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

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


	    NVIC_InitTypeDef NVIC_InitStructure;

	    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);


	    GPIO_InitTypeDef  GPIO_InitStructure;

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	    TIM3->CNT=0;
	    counter =0x80008000;

	    uint8_t i=0;
	    for(i=0;i<PERIOD_QUEUE_LENGTH;i++)
	    {
	    	p_queue.period[i]=0x0;
	    }
	    p_queue.counter=0x0;


	    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	    TIM_Cmd(TIM3, ENABLE);
	    Freq_Measure_Init();
	    xTaskCreate(DOL_Process,(signed char*)"DOL_PROCESS",128, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void DOL_Process( void *pvParameters )//
{
	static uint8_t period_overload_flag=0x1;
	uint8_t i=0;
	task_watches[DOL_TASK].task_status=TASK_IDLE;

	while(1)
	{
		task_watches[DOL_TASK].task_status=TASK_ACTIVE;

		channels[0].channel_data=counter+TIM3->CNT;//

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

			//sum_period=sum_period/PERIOD_QUEUE_LENGTH;
			uint32_t freq_div_10=0;
			uint32_t freq=(((MEASURE_TIM_PERIOD*10)<<8)*PERIOD_QUEUE_LENGTH)/sum_period;


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

		task_watches[DOL_TASK].counter++;
		vTaskDelay(50);
	}
}
