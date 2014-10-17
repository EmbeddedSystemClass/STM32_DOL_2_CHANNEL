#include "proto.h"
#include "channels.h"
#include "watchdog.h"
#include "address_dev.h"


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

 extern struct Channel  channels[];

//-----------------------------------------------------------------------------------
uint8_t   DEV_NAME[DEVICE_NAME_LENGTH_SYM] ="<<ROTOR>>";
uint8_t   NOTICE[DEVICE_DESC_MAX_LENGTH_SYM]="<-- GEOSPHERA_2014 -->";
uint8_t   VERSION[DEVICE_VER_LENGTH_SYM] ="\x30\x30\x30\x30\x31";

uint8_t   ADRESS_DEV=0x5;

uint8_t   dev_desc_len=20;
//--------------------------------global variable------------------------------------

uint8_t   	recieve_count;
uint8_t  	transf_count;
uint8_t  	buf_len;

uint8_t   crc_n_ERR;
uint8_t   COMMAND_ERR;

uint8_t   CUT_OUT_NULL;
uint8_t   PROTO_HAS_START;//была стартовая последовательность D7 29
uint8_t   frame_len=0;
//--------------------------------------------------------------------

uint8_t    RecieveBuf[256];
uint8_t    TransferBuf[256];

//--------------------------------------------------------------------
uint8_t    STATE_BYTE=0xC0;

uint16_t fr_err=0;

uint8_t  symbol=0xFF;

uint8_t	proto_type=PROTO_TYPE_NEW;
//-----------------------------------------------------------------------------------
union
{
	float result_float;
	uint8_t result_char[4];
}
sym_8_to_float;

xSemaphoreHandle xProtoSemaphore;
extern struct task_watch task_watches[];

#define RS_485_RECEIVE  USART_RS485_GPIO->BSRRH|=USART_RS485_DE; USART_RS485_GPIO->BSRRH|=USART_RS485_RE;
#define RS_485_TRANSMIT USART_RS485_GPIO->BSRRL|=USART_RS485_DE; USART_RS485_GPIO->BSRRL|=USART_RS485_RE;
//----------------------------------------------------------------------------------
uint8_t Send_Info(void);
uint8_t Node_Full_Init(void);
uint8_t Channel_List_Init(void);
uint8_t Channel_Get_Data(void);
uint8_t Channel_Set_Parameters(void);
uint8_t Channel_Set_Order_Query(void);
uint8_t Channel_Get_Data_Order(void);
uint8_t Channel_Set_State(void);
uint8_t Channel_Get_Data_Order_M2(void);
uint8_t Channel_Set_Reset_State_Flags(void);
uint8_t Channel_All_Get_Data(void);
uint8_t Channel_Set_Address_Desc(void);
uint8_t Channel_Set_Calibrate(void);
uint8_t Request_Error(uint8_t error_code);//


void ProtoBufHandling(void);
void Store_Dev_Address_Desc(void);
void Restore_Dev_Address_Desc(void);

uint8_t  CRC_Check( uint8_t *Spool,uint8_t Count);

void ProtoProcess( void *pvParameters );//
//----------------------------------------------------------------------------------
void /*USART_RS485_IRQHandler*/USART3_IRQHandler(void)
{
 	static portBASE_TYPE xHigherPriorityTaskWoken;
 	xHigherPriorityTaskWoken = pdFALSE;


 	if(USART_GetITStatus(USART_RS485, USART_IT_RXNE) != RESET)
   	{
 		USART_ClearITPendingBit(USART_RS485, USART_IT_RXNE);
   		symbol=(uint16_t)(USART_RS485->DR & (uint16_t)0x01FF);

   		if(recieve_count>MAX_LENGTH_REC_BUF)
   		{
   			recieve_count=0x0;
   			return;
   		}

		switch(proto_type)
		{
			case PROTO_TYPE_OLD:
			{
//				if(symbol==':')
//				{
//					recieve_count=0x0;
//				}
//
//				tab.tablo_proto_buf[recieve_count]=symbol;
//				recieve_count++;
//
//				if(recieve_count>1)
//				{
//					if(tab.tablo_proto_buf[1]==(recieve_count-2))//
//					{
//						 USART_ITConfig(USART_RS485, USART_IT_RXNE , DISABLE);
//						xSemaphoreGiveFromISR( xProtoSemaphore, &xHigherPriorityTaskWoken );
//
//						 if( xHigherPriorityTaskWoken != pdFALSE )
//						 {
//							portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//						 }
//					}
//				}
			}
			break;

			case PROTO_TYPE_NEW:
			{
			//--------------------------

				switch(symbol)
				{
					case (char)(0xD7):
					{
						RecieveBuf[recieve_count]=symbol;
						recieve_count++;
						CUT_OUT_NULL=1;
					}
					break;

					case (char)(0x29):
					{
						if(CUT_OUT_NULL==1)
						{
							RecieveBuf[0]=0x0;
							RecieveBuf[1]=0xD7;
							RecieveBuf[2]=0x29;
							PROTO_HAS_START=1;
							recieve_count=0x3;
						}
						else
						{
							RecieveBuf[recieve_count]=symbol;
							recieve_count++;
						}
						CUT_OUT_NULL=0;
					}
					break;

					case (char)(0x0):
					{
						if(CUT_OUT_NULL==1)	  //если после 0xD7-пропускаем
						{
							CUT_OUT_NULL=0;
						}
						else
						{
							RecieveBuf[recieve_count]=symbol;
							recieve_count++;
						}
					}
					break;

					default :
					{
						if(PROTO_HAS_START)
						{
							RecieveBuf[recieve_count]=symbol;
							recieve_count++;
							CUT_OUT_NULL=0;
						}
					}
				}

			   if(recieve_count>6)
			   {
					  if(recieve_count==6+frame_len)	  // принимаем указанное в frame_len число байт
					  {
							 USART_ITConfig(USART_RS485, USART_IT_RXNE , DISABLE);

							 xSemaphoreGiveFromISR( xProtoSemaphore, &xHigherPriorityTaskWoken );

							 if( xHigherPriorityTaskWoken != pdFALSE )
							 {
								portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
							 }
							 CUT_OUT_NULL=0;
							 PROTO_HAS_START=0;
					  }
			   }
			   else
			   {
					   if(recieve_count==6)
					   {
							frame_len=RecieveBuf[recieve_count-1];  // получаем длину данных после заголовка
					   }
			   }
			}
		}

   	}
   //-----------------------------------------------------------------------------------------------------------------
   	if(USART_GetITStatus(USART_RS485, USART_IT_TC) != RESET)
   	{

   		USART_ClearITPendingBit(USART_RS485, USART_IT_TC);

   		if(transf_count<buf_len)
   		{
   			if(transf_count<3)
   			{
   				USART_RS485->DR =TransferBuf[transf_count];
   				transf_count++;
   			}
   			else
   			{
   					if(CUT_OUT_NULL==0)
   					{
   						if(TransferBuf[transf_count]==(uint8_t)0xD7)
   						{
   							CUT_OUT_NULL=0x1;
   						}
   						USART_RS485->DR =TransferBuf[transf_count];
   						transf_count++;
   					}
   					else
   					{
   						USART_RS485->DR =(uint8_t)0x0 ;
   						CUT_OUT_NULL=0;
   					}
   			}
   		}
   		else
   		{
   			transf_count=0;
   			recieve_count=0;

   			CUT_OUT_NULL=0;
   			USART_ITConfig(USART_RS485, USART_IT_RXNE , ENABLE);
   			RS_485_RECEIVE;
   		}

   	}
   	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
//------------------------------------------------------------------------------
void Proto_Init(uint8_t init_type) //
{
		GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
		USART_InitTypeDef USART_InitStruct; // this is for the USART_RS485 initilization
		NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

		RCC_APB1PeriphClockCmd(RCC_USART_RS485, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_USART_RS485_GPIO, ENABLE);

		ADRESS_DEV=Address_Dev_Init();

		GPIO_InitStruct.GPIO_Pin = USART_RS485_TXD | USART_RS485_RXD; // Pins  (TX) and (RX) are used
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
		GPIO_Init(USART_RS485_GPIO, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

		GPIO_PinAFConfig(USART_RS485_GPIO, USART_RS485_TXD_PIN_SOURCE, GPIO_AF_USART_RS485); //
		GPIO_PinAFConfig(USART_RS485_GPIO, USART_RS485_RXD_PIN_SOURCE, GPIO_AF_USART_RS485);

		USART_InitStruct.USART_BaudRate = 57600;				// the baudrate is set to the value we passed into this init function
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
		USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
		USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
		USART_Init(USART_RS485, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	    GPIO_InitStruct.GPIO_Pin   = USART_RS485_DE|USART_RS485_RE;
	    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStruct.GPIO_PuPd =  GPIO_PuPd_NOPULL;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(USART_RS485_GPIO, &GPIO_InitStruct);

		USART_ClearFlag(USART_RS485, USART_FLAG_CTS | USART_FLAG_LBD  | USART_FLAG_TC  | USART_FLAG_RXNE );


		USART_ITConfig(USART_RS485, USART_IT_TC, ENABLE);
		USART_ITConfig(USART_RS485, USART_IT_RXNE , ENABLE);

		USART_Cmd(USART_RS485, ENABLE);

		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_3 );

		   /* Enabling interrupt from USART */
		NVIC_InitStructure.NVIC_IRQChannel = USART_RS485_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_EnableIRQ(USART_RS485_IRQn);						 // the properties are passed to the NVIC_Init function which takes care of the low level stuff



		RS_485_RECEIVE;


		crc_n_ERR=0x0;
		COMMAND_ERR=0x0;

		ChannelsInit();

		recieve_count=0x0;
		transf_count=0x0;
		buf_len=0x0;

		CUT_OUT_NULL=0;
		PROTO_HAS_START=0;

		if(init_type==PROTO_FIRST_INIT)
		{
			xTaskCreate(ProtoProcess,(signed char*)"PROTO",256,NULL, tskIDLE_PRIORITY + 1, NULL);
			task_watches[PROTO_TASK].task_status=TASK_ACTIVE;
			vSemaphoreCreateBinary( xProtoSemaphore );
		}
		return;
}
//-----------------------------------------------------------------------------
uint8_t Send_Info(void) //
{
	   uint8_t    i=0;

	   TransferBuf[0]=0x00;
	   TransferBuf[1]=0xD7;
	   TransferBuf[2]=0x29;
	   //------------------
	   TransferBuf[3]=ADRESS_DEV;  //
	   TransferBuf[4]=GET_DEV_INFO_RESP;  //
	   TransferBuf[6]=STATE_BYTE;

	   for(i=0;i<20;i++)
	   {				  //
		   if(i<DEVICE_NAME_LENGTH_SYM)
		   {
				TransferBuf[i+7]=DEV_NAME[i];
		   }
		   else
		   {
				TransferBuf[i+7]=0x00;
		   }
	   }

	   for(i=0;i<5;i++)                   //
	   {
	       if(i<DEVICE_VER_LENGTH_SYM)
		   {
		    	 TransferBuf[i+27]=VERSION[i];
		   }
	   }

	   TransferBuf[32]=CHANNEL_NUMBER;

	   for(i=0;i<CHANNEL_NUMBER;i++)
       {
		  	TransferBuf[i*2+33]=((channels[i].settings.set.type)<<4)|channels[i].settings.set.modific; //
		  	TransferBuf[i*2+33+1]=0x00;
	   }
	   for(i=0;i<dev_desc_len;i++)
	   {
			 TransferBuf[i+33+CHANNEL_NUMBER*2]=NOTICE[i];
	   }

	   TransferBuf[5]=28+CHANNEL_NUMBER*2+dev_desc_len;
	   TransferBuf[33+CHANNEL_NUMBER*2+dev_desc_len]=CRC_Check(&TransferBuf[1],32+CHANNEL_NUMBER*2+dev_desc_len); //

	   return (34+CHANNEL_NUMBER*2+dev_desc_len);
}
//-----------------------------------------------------------------------------
uint8_t Node_Full_Init(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_List_Init(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_Get_Data(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t  Channel_Set_Parameters(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_Set_Order_Query(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_Get_Data_Order(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_Set_State(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t  Channel_Get_Data_Order_M2(void) //
{
	return 0;
}
//-----------------------------------------------------------------------------
uint8_t Channel_Set_Reset_State_Flags(void) //
{
	STATE_BYTE=0x40;
	return	Request_Error(FR_SUCCESFUL);//
}
//-----------------------------------------------------------------------------
uint8_t Channel_All_Get_Data(void) //
{
	   uint8_t  index=0,i=0;

	   TransferBuf[0]=0x00;TransferBuf[1]=0xD7;TransferBuf[2]=0x29;
	   TransferBuf[3]=ADRESS_DEV;
	   TransferBuf[4]=CHANNEL_ALL_GET_DATA_RESP;
	   TransferBuf[6]=STATE_BYTE;
	    for(i=0;i<CHANNEL_NUMBER;i++)
	    {
			  TransferBuf[index+7]=i;
			  index++;
			  TransferBuf[index+7]=((channels[i].settings.set.type)<<4)|channels[i].settings.set.modific;
			  index++;

			  switch(channels[i].settings.set.type)
			  {
				 	case CHNL_DOL:	 //
					{
						  switch(channels[i].settings.set.modific)
					      {
								  case CHNL_DOL_ENC:
								  {
								          TransferBuf[index+7]=(uint8_t)((channels[i].channel_data)&0x000000FF); //
								          index++;

										  TransferBuf[index+7]=(uint8_t)(((channels[i].channel_data)&0x0000FF00)>>8);
										  index++;

								          TransferBuf[index+7]=(uint8_t)(((channels[i].channel_data)&0x00FF0000)>>16); //
								          index++;

										  TransferBuf[index+7]=(uint8_t)(((channels[i].channel_data)&0xFF000000)>>24);
										  index++;


										  TransferBuf[index+7]=channels[i].settings.set.state_byte_1;	 //
				                          index++;
								  }
								  break;
						   }
					}
					break;

				 case CHNL_FREQ: //
				 {
					  switch(channels[i].settings.set.modific)
					  {
							  case CHNL_FREQ_COUNT_T:
							  {
									  TransferBuf[index+7]=(uint8_t)((channels[i].channel_data)&0x000000FF); //
									  index++;
									  TransferBuf[index+7]=(uint8_t)(((channels[i].channel_data)&0x0000FF00)>>8);
									  index++;
									  TransferBuf[index+7]=(uint8_t)channels[i].settings.set.state_byte_1;	 //
									  index++;
							  }
							  break;

					  	  	  case CHNL_FREQ_256:
							  {
									  TransferBuf[index+7]=(uint8_t)((channels[i].channel_data)&0x000000FF); //
									  index++;
									  TransferBuf[index+7]=(uint8_t)(((channels[i].channel_data)&0x0000FF00)>>8);
									  index++;
									  TransferBuf[index+7]=(uint8_t)channels[i].settings.set.state_byte_1;	 //
									  index++;
							  }
							  break;
					   }
				  }
				  break;

				 	default:
				 	{
				 		return 0;
				 	}
			  }
		  }

		  TransferBuf[5]=index+2; 						 //
		  TransferBuf[index+7]=CRC_Check(&TransferBuf[1],(uint8_t)(index+7)-1); //
		  return (uint8_t)(7+index+1);
}

uint8_t Request_Error(uint8_t error_code) //
{
	TransferBuf[0]=0x00;TransferBuf[1]=0xD7;TransferBuf[2]=0x29;
    TransferBuf[3]=ADRESS_DEV;  //
    TransferBuf[7]=RecieveBuf[4]; //
    TransferBuf[4]=0xFF;  //

	TransferBuf[6]=STATE_BYTE; //
    TransferBuf[8]=error_code;
    TransferBuf[5]=0x04;	  //
    TransferBuf[9]=CRC_Check(TransferBuf,9);
	return 10;
}
//-----------------------------------------------------------------------------
void ProtoBufHandling(void) //
{
  switch(RecieveBuf[4])
  {
//---------------------------------------
  	case GET_DEV_INFO_REQ:
	{
		buf_len=Send_Info();
	}
	break;
//---------------------------------------
  	case NODE_FULL_INIT_REQ:
	{
		buf_len=Node_Full_Init();
	}
	break;
//---------------------------------------
  	case CHANNEL_LIST_INIT_REQ:
	{
		buf_len=Channel_List_Init();
	}
	break;
//---------------------------------------
	case CHANNEL_GET_DATA_REQ:
	{
		buf_len=Channel_Get_Data();
	}
	break;
	//-----------------------------------
	case CHANNEL_SET_PARAMETERS_REQ:
	{
		buf_len=Channel_Set_Parameters();
	}
	break;
	//-----------------------------------
	case CHANNEL_SET_ORDER_QUERY_REQ:
	{
		buf_len=Channel_Set_Order_Query();
	}
	break;
//----------------------------------------
	case CHANNEL_GET_DATA_ORDER_REQ:
	{
		 buf_len=Channel_Get_Data_Order();
	}
	break;
//----------------------------------------
	case CHANNEL_SET_STATE_REQ:
	{
		 buf_len=Channel_Set_State();
	}
	break;
//----------------------------------------
	case CHANNEL_GET_DATA_ORDER_M2_REQ:
	{
		 buf_len=Channel_Get_Data_Order_M2();
	}
	break;
//------------------------------------------
	case CHANNEL_SET_RESET_STATE_FLAGS_REQ:
	{
		buf_len=Channel_Set_Reset_State_Flags();
	}
	break;
//------------------------------------------
	case CHANNEL_ALL_GET_DATA_REQ:
	{
		 buf_len=Channel_All_Get_Data();
	}
	break;
//------------------------------------------
/*	case CHANNEL_SET_ADDRESS_DESC:
	{
		 buf_len=Channel_Set_Address_Desc();
	}
	break;
//------------------------------------------
	case CHANNEL_SET_CALIBRATE:
	{
		 buf_len=Channel_Set_Calibrate();
	}
	break;*/
//------------------------------------------
    default:
	{
       COMMAND_ERR=0x1;//
	   buf_len=Request_Error(FR_COMMAND_NOT_EXIST);
    }
  }

  return;
}
//-----------------------------------------------------------------------------------
void ProtoProcess( void *pvParameters )
{
	uint8_t   crc_n;
	task_watches[PROTO_TASK].task_status=TASK_IDLE;
	while(1)
	{
		task_watches[PROTO_TASK].task_status=TASK_IDLE;
		if( xProtoSemaphore != NULL )
		{

			if( xSemaphoreTake( xProtoSemaphore, ( portTickType ) PROTO_STANDBY_TIME ) == pdTRUE )
			{
				task_watches[PROTO_TASK].task_status=TASK_ACTIVE;
				switch(proto_type)
				{
					case PROTO_TYPE_OLD:
					{

					}
					break;

					case PROTO_TYPE_NEW:
					{
						crc_n=RecieveBuf[recieve_count-1];
						transf_count=0;
						if((CRC_Check(RecieveBuf,(recieve_count-CRC_LEN))==crc_n)&& (RecieveBuf[3]==ADRESS_DEV))
						{
							ProtoBufHandling();//

							transf_count=0;
							recieve_count=0;
							CUT_OUT_NULL=0;
							RS_485_TRANSMIT;
							USART_RS485->DR =TransferBuf[transf_count];
							transf_count++;//
						}
						else
						{
							crc_n_ERR=0x1;//
							USART_ITConfig(USART_RS485, USART_IT_RXNE , ENABLE);
						}
					}
					break;
				}
				task_watches[PROTO_TASK].counter++;
			}
			else
			{
				 Proto_Init(PROTO_REINIT);
			}
		}
		task_watches[PROTO_TASK].task_status=TASK_IDLE;
	}
}
//-----------------------crc_n------------------------------------------------------------
uint8_t  CRC_Check( uint8_t  *Spool_pr,uint8_t Count_pr )
{
	uint8_t crc_n = 0;
	uint8_t  *Spool;
	uint8_t  Count ;

	Spool=Spool_pr;
	Count=Count_pr;

  		while(Count!=0x0)
        {
	        crc_n = crc_n ^ (*Spool++);//

	        crc_n = ((crc_n & 0x01) ? (uint8_t)0x80: (uint8_t)0x00) | (uint8_t)(crc_n >> 1);

	        if (crc_n & (uint8_t)0x80) crc_n = crc_n ^ (uint8_t)0x3C;
			Count--;
        }
    return crc_n;
}
 //-----------------------------------------------------------------------------------------------

