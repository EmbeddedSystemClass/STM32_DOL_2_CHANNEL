#include "channels.h"
struct Channel  channels[CHANNEL_NUMBER];
//-----------------------------------
void ChannelsInit(void) //
{
	channels[0].number=0;		  //
	channels[0].settings.set.type=CHNL_DOL;
	channels[0].settings.set.modific=CHNL_DOL_ENC;
	channels[0].settings.set.state_byte_1=0x40;
	channels[0].settings.set.state_byte_2=0x0A;
	channels[0].channel_data=0x80008000;
	channels[0].channel_data_calibrate=0;
	channels[0].calibrate.cal.calibrate=0;

	channels[1].number=1;		  //
	channels[1].settings.set.type=CHNL_DOL;
	channels[1].settings.set.modific=CHNL_DOL_ENC;
	channels[1].settings.set.state_byte_1=0x40;
	channels[1].settings.set.state_byte_2=0x0A;
	channels[1].channel_data=0x80008000;
	channels[1].channel_data_calibrate=0;
	channels[1].calibrate.cal.calibrate=0;
	return;
}

