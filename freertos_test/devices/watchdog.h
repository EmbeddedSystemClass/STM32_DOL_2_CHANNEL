#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "stm32f4xx.h"

#define TASK_NUM	2//количество задач, кроме задачи ватчдога , в системе

enum
{
	PROTO_TASK=0,
	DOL_TASK=1,
};

enum
{
	TASK_ACTIVE=0,
	TASK_IDLE=1
};

struct task_watch
{
	uint32_t counter;
	uint8_t  task_status;
};

void Watchdog_Init(void);

#endif
