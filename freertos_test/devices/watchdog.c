#include "watchdog.h"
#include "stm32f4xx_iwdg.h"
//FreeRTOS:

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define LSI_FREQ	40000

struct task_watch task_watches[TASK_NUM];
static void Watchdog_Task(void *pvParameters);//

void Watchdog_Init(void)
{
	uint8_t i;

	for(i=0;i<TASK_NUM;i++)
	{
		task_watches[i].task_status=TASK_IDLE;
	}


	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
	     dispersion) */
	  /* Enable write access to IWDG_PR and IWDG_RLR registers */


	  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	  /* IWDG counter clock: LSI/32 */
	  IWDG_SetPrescaler(IWDG_Prescaler_128);

	  /* Set counter reload value to obtain 250ms IWDG TimeOut.
	     Counter Reload Value = 250ms/IWDG counter clock period
	                          = 250ms / (LSI/32)
	                          = 0.25s / (LsiFreq/32)
	                          = LsiFreq/(32 * 4)
	                          = LsiFreq/128
	   */
	  IWDG_SetReload(LSI_FREQ/64);

	  /* Reload IWDG counter */
	  IWDG_ReloadCounter();

	  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	  IWDG_Enable();

	  xTaskCreate(Watchdog_Task,(signed char*)"INIT",128,NULL, tskIDLE_PRIORITY + 1, NULL);
}


static void Watchdog_Task(void *pvParameters)
{
	while(1)
	{
		if(((task_watches[PROTO_TASK].counter>0)||(task_watches[PROTO_TASK].task_status==TASK_IDLE))&&
		   ((task_watches[DOL_TASK].counter>0)	||(task_watches[DOL_TASK].task_status==TASK_IDLE)))
		{//проверка счетчиков
			IWDG_ReloadCounter();
		}

		task_watches[PROTO_TASK].counter=0;
		task_watches[DOL_TASK].counter=0;
		vTaskDelay(500);
	}
}
