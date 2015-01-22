#ifndef HALL_SENSORS_H
#define HALL_SENSORS_H

enum
{
	HALL_STATE_0=0x3,
	HALL_STATE_1=0x2,
	HALL_STATE_2=0x0,
	HALL_STATE_3=0x1,
};


#define IDR_MASK	0xF
#define ENC_MASK 	0x3

#define HALL_SENSORS_PORT    		GPIOA
#define HALL_SENSORS_PORT_RCC 		RCC_AHB1Periph_GPIOA
#define HALL_SENSORS_PORT_EXTI 		EXTI_PortSourceGPIOA

#define HALL_SENSOR_0_PIN 			GPIO_Pin_6
#define HALL_SENSOR_1_PIN 			GPIO_Pin_7
#define HALL_SENSOR_2_PIN 			GPIO_Pin_8
#define HALL_SENSOR_3_PIN 			GPIO_Pin_9

#define HALL_SENSOR_0_PinSource 	GPIO_PinSource6
#define HALL_SENSOR_1_PinSource 	GPIO_PinSource7
#define HALL_SENSOR_2_PinSource 	GPIO_PinSource8
#define HALL_SENSOR_3_PinSource 	GPIO_PinSource9

#define ENC_PORT    		GPIOC
#define ENC_PORT_RCC 		RCC_AHB1Periph_GPIOC

#define ENC_0_PIN 			GPIO_Pin_11
#define ENC_1_PIN 			GPIO_Pin_12


void Hall_Sensors_Init(void);


#endif
