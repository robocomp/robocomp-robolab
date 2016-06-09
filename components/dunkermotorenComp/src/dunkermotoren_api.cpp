#include "dunkermotoren_api.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <QString>
#include <QDebug>


void Dunker_printMessageData(VSCAN_MSG msg)
{
	printf("dunkermotoren_api::printMessageData() %#02lx, %#02X | %#02x _ %#02x %#02x _ %#02x _ %#02x  %#02x %#02x %#02x \n",
	msg.Id,
	msg.Size,
    msg.Data[0],
    msg.Data[1],
    msg.Data[2],
    msg.Data[3],
    msg.Data[4],
    msg.Data[5],
    msg.Data[6],
    msg.Data[7] );
}

uint8_t Dunker_getCommandSpecifier(bool _write, bool _expedited, bool _sizeSpecified)
{
	uint8_t commandSpecifier = 0x00;
	uint8_t write = 0x20;
	uint8_t read = 0x40;
	uint8_t expedited = 0x02;
	uint8_t sizeSpecified = 0x01;
	if(_write)
		commandSpecifier|=write;
	else
		commandSpecifier|=read;
	if(_expedited)
		commandSpecifier|=expedited;
	if(_sizeSpecified)
		commandSpecifier|=sizeSpecified;
	return commandSpecifier;
}

uint8_t Dunker_getDefaultReadCommandSpecifier(void)
{
	uint8_t commandSpecifier = 0x00;
	uint8_t read = 0x40;
	commandSpecifier|=read;
	return commandSpecifier;
}

int Dunker_getReadedData(VSCAN_MSG msg)
{
	int result = 0;
	result += msg.Data[7]<<24;
	result += msg.Data[6]<<16;
	result += msg.Data[5]<<8;
	result += msg.Data[4];
	return result;
}

uint8_t Dunker_getDefaultWriteCommandSpecifier(void)
{
	uint8_t commandSpecifier = 0x00;
	uint8_t write = 0x20;
	uint8_t expedited = 0x02;
	uint8_t sizeSpecified = 0x01;
	commandSpecifier|=write;
	commandSpecifier|=expedited;
	commandSpecifier|=sizeSpecified;
	return commandSpecifier;
}

VSCAN_MSG Dunker_getMessageData(uint8_t nodeId, uint8_t CommandSpecifier, uint16_t obj_id, uint8_t obj_subid, uint32_t obj_data)
{
	VSCAN_MSG msg;
	msg.Flags = VSCAN_FLAGS_STANDARD;
	msg.Id = 0X600;
	msg.Id |= nodeId;
	msg.Size = 8;
	msg.Data[0] = CommandSpecifier;
	msg.Data[1] = obj_id&0x00FF;
	msg.Data[2] = (obj_id&0xFF00)>>8;
	msg.Data[3] = obj_subid;
	msg.Data[4] = (obj_data&0x000000FF);
	msg.Data[5] = (obj_data&0x0000FF00)>>8;
	msg.Data[6] = (obj_data&0x00FF0000)>>16;
	msg.Data[7] = (obj_data&0xFF000000)>>24;
	return msg;
}

int Dunker_writeWaitReadMessage(VSCAN_HANDLE Handle, VSCAN_MSG* msg)
{
	int status;
	DWORD   written;
	//~ printf("------------------------------------\n");
	//~ printf("Write messages\n");
	//~ printMessageData(*msg);

    status = VSCAN_Write(Handle,msg,1,&written);
    //~ printf("Flush messages\n");
    status = VSCAN_Flush(Handle);
	
	//~ VSCAN_GetErrorString(status,string,32);
    //~ printf("%s\n",string);
    //printf("%d", written);

	DWORD read;
	VSCAN_MSG aux_msg;
	memcpy(&aux_msg, msg, sizeof(VSCAN_MSG));
	
	//~ printf("Read messages\n");
	int retries=0;
	status=VSCAN_Read(Handle, msg, 1, &read);
	//~ printf("----------------------------------------0\n");
	//~ Dunker_printMessageData(*msg);
	//~ Dunker_printMessageData(aux_msg);
	while( status != 0 || aux_msg.Data[1] != msg->Data[1] || aux_msg.Data[2] != msg->Data[2] || aux_msg.Data[3] != msg->Data[3])
	{
		//~ printf("----------------------------------------1\n");
		if(msg->Data[1] != 0 || msg->Data[2] != 0 || msg->Data[3] != 0)
		{
			printf("dunkermotoren_api::writeWaitReadMessage() Read status = %d: ", status);
			Dunker_printMessageData(*msg);
		}
		if((aux_msg.Data[1] != msg->Data[1] || aux_msg.Data[2] != msg->Data[2] || aux_msg.Data[3] != msg->Data[3]) && (msg->Data[1] != 0 || msg->Data[2] != 0 || msg->Data[3] != 0))
		{
			printf("dunkermotoren_api::writeWaitReadMessage() Warning: (Retries %d) d unspeckted response: ", retries);
			printf("dunkermotoren_api::writeWaitReadMessage() aux_msg.Data[1](%#x) != msg->Data[1]((%#x)) || aux_msg.Data[2]((%#x)) != msg->Data[2]((%#x) || aux_msg.Data[3](%#x) != msg->Data[3](%#x)\n", aux_msg.Data[1], msg->Data[1], aux_msg.Data[2], msg->Data[2], aux_msg.Data[3], msg->Data[3]);
			Dunker_printMessageData(*msg);
		}
		usleep(2000);
		status=VSCAN_Read(Handle, msg, 1, &read);
		//~ Dunker_printMessageData(*msg);
		//~ Dunker_printMessageData(aux_msg);
		if(retries++>MAX_READ_RETRIES)
		{
			printf("dunkermotoren_api::writeWaitReadMessage() ERROR: No se pudo leer la respuesta al comando enviado (retries = %d)\n", retries);
			return -1;
		}
		//~ printf("----------------------------------------2\n");
	}
	//~ printf("%d == %d -- %d == %d -- %d == %d\n", aux_msg.Data[1], msg->Data[1], aux_msg.Data[2], msg->Data[2], aux_msg.Data[3], msg->Data[3]);
	//~ printf("Read status = %d\n", status);
	//~ printMessageData(*msg);
	
	
	//~ printf("------------------------------------\n");
	return status;
}


int Dunker_multiWriteWaitReadMessage(VSCAN_HANDLE Handle, VSCAN_MSG* msgs, int msg_count)
{
	int status;
	int ok_readed=0;
	DWORD   written;
	char string[100];
	uint8_t mcount;
	int read_result=0;
	//~ printf("------------------------------------\n");
	
	VSCAN_MSG aux_msgs[msg_count];
	VSCAN_MSG readed_msgs[msg_count];
	memcpy(aux_msgs, msgs, sizeof(VSCAN_MSG)*msg_count);
	memcpy(readed_msgs, msgs, sizeof(VSCAN_MSG)*msg_count);
	//~ printf("Initial messages:\n");
	//~ for(mcount=0;mcount<msg_count;mcount++)
	//~ {
		//~ Dunker_printMessageData(msgs[mcount]);
	//~ }

    status = VSCAN_Write(Handle,msgs,msg_count,&written);
    //printf("Flush messages\n");
    status = VSCAN_Flush(Handle);
	
	memset(string, '\0', 100);
	VSCAN_GetErrorString(status,string,32);
    //printf("%s\n",string);
    //printf("%ld\n", written);

	DWORD read;
	bool cmds_readed[msg_count];
	for(int x=0;x<msg_count;x++)
	{
		cmds_readed[x]=false;
	}
	
	

	int retries=0;
	
	status=VSCAN_Read(Handle, readed_msgs, msg_count, &read);
	//~ printf("status = %d, %ld\n",status, read);
	while( status !=0 || read_result!=1 || ok_readed != msg_count)
	{
		//~ printf("Retries %d.\n", retries);
		if(read>0)
		{
			//printf("Out condition status(%d) !=0 || read_result(%d)!=1 || ok_readed(%d) != msg_count(%d)\n",status, read_result, ok_readed, msg_count);
			read_result=1;
			for(mcount=0; mcount<read; mcount++)
			{
				//~ Dunker_printMessageData(msgs[mcount]);
				for(int oldc=0;oldc<msg_count;oldc++)
				{
					//printf("prueba %#02lx != %#02lx = %d\n", aux_msgs[oldc].Id%16, msgs[mcount].Id%16, aux_msgs[oldc].Id%16 != msgs[mcount].Id%16);
					read_result = !((aux_msgs[oldc].Id%0x10 != readed_msgs[mcount].Id%0x10 || aux_msgs[oldc].Data[1] != readed_msgs[mcount].Data[1] || aux_msgs[oldc].Data[2] != readed_msgs[mcount].Data[2] || aux_msgs[oldc].Data[3] != readed_msgs[mcount].Data[3] || aux_msgs[oldc].Id%16 != readed_msgs[mcount].Id%16) && (aux_msgs[oldc].Data[1] != 0 || aux_msgs[oldc].Data[2] != 0 || aux_msgs[oldc].Data[3] != 0));
					//printf("!((%#x != %#x || %#x != %#x || %#x != %#x) && (%#x != 0 || %#x != 0 || %#x != 0)) = %d\n", aux_msgs[oldc].Data[1], msgs[mcount].Data[1], aux_msgs[oldc].Data[2], msgs[mcount].Data[2], aux_msgs[oldc].Data[3], msgs[mcount].Data[3], aux_msgs[oldc].Data[1], aux_msgs[oldc].Data[2], aux_msgs[oldc].Data[3], !((aux_msgs[oldc].Data[1] != msgs[mcount].Data[1] || aux_msgs[oldc].Data[2] != msgs[mcount].Data[2] || aux_msgs[oldc].Data[3] != msgs[mcount].Data[3]) && (aux_msgs[oldc].Data[1] != 0 || aux_msgs[oldc].Data[2] != 0 || aux_msgs[oldc].Data[3] != 0)));
					if(read_result != 0)
					{
						ok_readed++;
						//~ printf("found = %d\n", ok_readed);
						retries=0;
						cmds_readed[oldc]=true;
						memcpy(&msgs[oldc], &readed_msgs[mcount], sizeof(VSCAN_MSG));
						break;
					}
				}
			}
			//printf("Read result = %d\n", read_result);
		}	
		usleep(20000);
		status=VSCAN_Read(Handle, readed_msgs, msg_count, &read);
		//printf("status = %d, %ld\n",status, read);
		if(retries++>MAX_READ_RETRIES)
		{
			printf("dunkermotoren_api::multiWriteWaitReadMessage() ERROR: No se pudo leer la respuesta al comando enviado (retries = %d)\n", retries);
			for(int x=0;x<msg_count;x++)
			{
				if(cmds_readed[x]==false)
				{
					printf("dunkermotoren_api::multiWriteWaitReadMessage() ERROR on command %d\n", x);
					Dunker_printMessageData(aux_msgs[x]);
					Dunker_printMessageData(msgs[x]);
				}
			}
			return -1;
		}
	}
	//~ printf("%d == %d -- %d == %d -- %d == %d\n", aux_msg.Data[1], msg->Data[1], aux_msg.Data[2], msg->Data[2], aux_msg.Data[3], msg->Data[3]);
	//~ printf("Read status = %d\n", status);
	//~ printMessageData(*msg);
	
	
	//~ printf("......................\n");
	return status;
}


int Dunker_setBaudRate(VSCAN_HANDLE Handle, int baudRate)
{
	void * speed;
	switch(baudRate)
	{

		case 20000:
			speed=VSCAN_SPEED_20K;
		break;
		case 50000:
			speed=VSCAN_SPEED_50K;
		break;
		case 100000:
			speed=VSCAN_SPEED_100K;
		break;
		case 125000:
			speed=VSCAN_SPEED_125K;
		break;
		case 250000:
			speed=VSCAN_SPEED_250K;
		break;
		case 500000:
			speed=VSCAN_SPEED_500K;
		break;
		case 800000:
			speed=VSCAN_SPEED_800K;
		break;
		case 1000000:
			speed=VSCAN_SPEED_1M;
		break;
		default:
			printf("dunkermotoren_api::setBaudRate() ERROR: %d no es un valor de baudRate válido (20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000)\n", baudRate);
			return -1;
	}
	int status = VSCAN_Ioctl(Handle, VSCAN_IOCTL_SET_SPEED, (void *)speed);
	return status;
}


int Dunker_disablePower(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	//Disable power stage
	VSCAN_MSG msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3004,0x00,0x00000000);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_enablePower(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	//Enable power stage
	VSCAN_MSG msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3004, 0x00, 0x00000001);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_openBreak(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	//~if the motor has a brake, this command opens the brake (just in case)
	VSCAN_MSG msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3150,0x00,0x00000001);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_clearError(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	//clear error (just in case)
	VSCAN_MSG msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3000,0x00,0x00000001);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	printf("dunkermotoren_api::clearError()\n");
	return status;
}

int Dunker_setOperationMode(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t mode)
{
	//set operation mode
	VSCAN_MSG msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3003,0x00,mode);
	printf("dunkermotoren_api::setOperationMode():  Setting operation mode: \n");
	Dunker_printMessageData(msg);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_getOperationMode(VSCAN_HANDLE Handle, uint8_t NodeId, int* mode)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3003, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*mode = Dunker_getReadedData(msg);
	return status;
}

//~ int Dunker_setDinamicLimitations(VSCAN_HANDLE Handle, uint8_t NodeId)
//~ {
	//~ VSCAN_MSG msgs[7];
	//~ //Current Dynamic limitation
	//~ //CurrDynLimitOff()
	//~ msgs[0] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000000);
	//~ //Curr Limit Pos
	//~ msgs[1] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3221,0x00,60000);
	//~ //Curr Limit Neg
	//~ msgs[2] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3223,0x00,60000);
	//~ //CurrDynLimitOn() - Dynamic Current limitation On
	//~ msgs[3] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000001);
	//~ //CurrDynLimitPeak(mA) - Peak current
	//~ msgs[4] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x01, 60000);
	//~ //CurrDynLimitCont(mA) - Continous current
	//~ msgs[5] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x02, 10000);
	//~ //CurrDynLimitTime(ms) - Time for Peak current
	//~ msgs[6] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x03, 2000);
	//~ int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 7);
	//~ return status;
//~ }



int Dunker_setPositiveCurrentLimit(VSCAN_HANDLE Handle, uint8_t NodeId, int limit)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3221, 0x00, limit);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}


int Dunker_syncPositiveCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3221, 0x00, limit[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_setNegativeCurrentLimit(VSCAN_HANDLE Handle, uint8_t NodeId, int limit)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3223,0x00, limit);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncNegativeCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3223, 0x00, limit[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_setDynCurrentLimitOff(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000000);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncDynCurrentLimitOff(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000000);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_setDynCurrentLimitOn(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000001);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncDynCurrentLimitOn(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000001);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_setDynCurrentLimitPeak(VSCAN_HANDLE Handle, uint8_t NodeId, int limit)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x01, limit);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncDynCurrentLimitPeak(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x01, limit[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_setDynCurrentLimitContinous(VSCAN_HANDLE Handle, uint8_t NodeId, int limit)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x02, limit);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncDynCurrentLimitContinous(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x02, limit[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_setDynCurrentLimitTime(VSCAN_HANDLE Handle, uint8_t NodeId, int time)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x03, time);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncDynCurrentLimitTime(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* time)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x03, time[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetEncoderRes(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* resol)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3962,0x00,resol[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_configureErrorsAndFeedback(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	printf("dunkermotoren_api::configureErrorsAndFeedback()\n");
	VSCAN_MSG msgs[11];
	
	//#############################################################################
	//Primary Velocity and Position Controller
	//VelKp
	msgs[0] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3310, 0x00, 100);
	//VelKp_Stop
	msgs[1] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3310, 0x01, 50);
	//VelKi
	msgs[2] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3311, 0x00, 0x00000000);
	//VelKd
	msgs[3] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3312, 0x00, 0x00000000);
	//VelKiLimit
	msgs[4] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3313, 0x00, 0x00000000);

	//Secondary Velocity Controller
	//sVelKp
	msgs[5] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3510, 0x00, 100);
	//sVelKi
	msgs[6] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3511, 0x00, 10);

	//Current Controller
	//curr Kp
	msgs[7] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3210, 0x00, 100);
	//curr Ki
	msgs[8] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3211, 0x00, 100);
	//Feed forward***************************************************************
	//Vel Feedforward
	msgs[9] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3314, 0x00, 1000);
	//set vel source from command
	msgs[10] = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3304, 0x00, 768);
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, 11);
	return status;
}


//set the maximum deviation between command and actual position
int Dunker_setMaxPosFollowingError(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t max_err)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3732, 0x00, max_err);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}



int Dunker_setActualPosition(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t pos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3762, 0x00, pos);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_clearActualPosition(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	int status = Dunker_setActualPosition(Handle, NodeId, 0);
	printf("dunkermotoren_api::clearActualPosition()\n");
	return status;
}

int Dunker_resetModule(VSCAN_HANDLE Handle, uint8_t NodeId)
{
	int status=0;
	//~ status = Dunker_setDinamicLimitations(Handle, NodeId);
	//~ if( status !=0 )
		//~ return status;
	Dunker_clearActualPosition(Handle, NodeId);
	if( status !=0 )
		return status;
	Dunker_configureErrorsAndFeedback(Handle, NodeId);
	if( status !=0 )
		return status;
	Dunker_clearError(Handle, NodeId);
	return status;
}

int Dunker_syncResetModule(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	int count;
	int status=0;
	printf("dunkermotoren_api::syncResetModule()\n");
	for(count=0;count<node_count;count++)
	{	
		int status=0;
		//~ status = Dunker_syncSetDinamicLimitations(Handle, node_count, NodeIds);
		//~ if( status !=0 )
			//~ return status;
		status = Dunker_syncClearActualPosition(Handle, node_count, NodeIds);
		if( status !=0 )
		{
			printf("dunkermotoren_api::syncResetModule(): ERROR syncClearActualPosition(err=%d)\n",status);
			return status;
		}
		status = Dunker_syncClearError(Handle, node_count, NodeIds);
		if( status !=0 )
		{
			printf("dunkermotoren_api::syncResetModule(): ERROR syncClearError (err=%d)\n",status);
			return status;
		}
		status = Dunker_syncConfigureErrorsAndFeedback(Handle, node_count, NodeIds);
		if( status !=0 )
		{
			printf("dunkermotoren_api::syncResetModule(): ERROR syncConfigureErrorsAndFeedback (err=%d)\n",status);
			return status;
		}
	}
	return status;
}

int Dunker_moveToAbsolutePosition(VSCAN_HANDLE Handle, uint8_t NodeId, int pos)
{
	VSCAN_MSG msg;
	printf("dunkermotoren_api::moveToAbsolutePosition(): Handle %d, node %#x to pos %d:\n",Handle, NodeId, pos);
	msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3790, 0x00, pos);
	Dunker_printMessageData(msg);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_moveToRelativePosition(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t pos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3791, 0x00, pos);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_setAcceleration(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t deltaV, uint32_t deltaT)
{
	VSCAN_MSG msgs[2];
	msgs[0] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3340, 0x00, deltaV);
	msgs[1] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3341, 0x00, deltaT);
	int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 2);
	return status;
}
 
int Dunker_setDecceleration(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t deltaV, uint32_t deltaT)
{
	VSCAN_MSG msgs[2];
	msgs[0] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3342, 0x00, deltaV);
	msgs[1] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3343, 0x00, deltaT);
	int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 2);
	return status;
}

int Dunker_getAcceleration(VSCAN_HANDLE Handle, uint8_t NodeId, int* deltaV, int* deltaT)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3340, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*deltaV = Dunker_getReadedData(msg);
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3341, 0x00);
	status = Dunker_writeWaitReadMessage(Handle, &msg);
	*deltaT = Dunker_getReadedData(msg);
	return status;
}
 
int Dunker_getDecceleration(VSCAN_HANDLE Handle, uint8_t NodeId, int* deltaV, int* deltaT)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3342, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*deltaV = Dunker_getReadedData(msg);
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3343, 0x00);
	status = Dunker_writeWaitReadMessage(Handle, &msg);
	*deltaT = Dunker_getReadedData(msg);
	return status;
}


int Dunker_getPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3762, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*pos = Dunker_getReadedData(msg);
	return status;
}

int Dunker_getActualCurrent(VSCAN_HANDLE Handle, uint8_t NodeId, int* curr)
{
	//actual current
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3113, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*curr = Dunker_getReadedData(msg);
	return status;
}

int Dunker_getVel(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel)
{
	//actual Velocity
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3A04, 0x01);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*vel = Dunker_getReadedData(msg);
	return status;
}

int Dunker_setVel(VSCAN_HANDLE Handle, uint8_t NodeId, int velocity)
{
	//set desired velocity "rpm"
// 	const int32_t vel = 30;
// 	printf("vel %d\n", velocity);
	VSCAN_MSG msg = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3300, 0x00, velocity);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

//Decimas de grado
int Dunker_getMotorTemperature(VSCAN_HANDLE Handle, uint8_t NodeId, int* temp)
{
	//actual Velocity
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3114, 0x03);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*temp = Dunker_getReadedData(msg);

	return status;
}

//Decimas de grado
int Dunker_getPowerStageTemperature(VSCAN_HANDLE Handle, uint8_t NodeId, int* stemp)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3114, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*stemp = Dunker_getReadedData(msg);
	return status;
}

int Dunker_changeNodeId(VSCAN_HANDLE Handle, uint8_t NodeId, uint8_t newNodeId)
{
	VSCAN_MSG msgs[2];
	msgs[0] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x2000, 0x01, 0x6E657277);
	msgs[1] = Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x2000, 0x02, newNodeId);
	int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 2);
	printf("dunkermotoren_api::changeNodeId(): NodeId changed. You MUST turn off and turn on again the device\n");
	return status;
}

int Dunker_setMaxPos(VSCAN_HANDLE Handle, uint8_t NodeId, int maxPos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3720, 0x01, maxPos);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncSetMaxPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* maxPos)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3720,0x01, maxPos[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_getMaxPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3720, 0x01);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*pos = Dunker_getReadedData(msg);
	return status;
}

int Dunker_setMinPos(VSCAN_HANDLE Handle, uint8_t NodeId, int minPos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3720, 0x00, minPos);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncSetMinPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* minPos)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3720,0x00, minPos[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_getMinPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3720, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*pos = Dunker_getReadedData(msg);
	return status;
}

//-------------------

int Dunker_setMaxVelPositive(VSCAN_HANDLE Handle, uint8_t NodeId, int vel)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3321, 0x00, vel);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncSetMaxVelPositive(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* vels)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3321, 0x00, vels[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_getMaxVelPositive(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3321, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*vel = Dunker_getReadedData(msg);
	return status;
}

int Dunker_setMaxVelNegative(VSCAN_HANDLE Handle, uint8_t NodeId, int vel)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultWriteCommandSpecifier(), 0x3323, 0x00, vel);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncSetMaxVelNegative(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* vels)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3323, 0x00, vels[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_getMaxVelNegative(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3323, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*vel = Dunker_getReadedData(msg);
	return status;
}

int Dunker_getMaxMotorVoltage(VSCAN_HANDLE Handle, uint8_t NodeId, int* vol)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3031, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*vol = Dunker_getReadedData(msg);
	return status;
}

int Dunker_getPower(VSCAN_HANDLE Handle, uint8_t NodeId, int* pow)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3111, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*pow = Dunker_getReadedData(msg);
	return status;
}

int Dunker_syncMoveToAbsolutePosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* pos)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3790, 0x00, pos[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncMoveToRelativePosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* pos)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3791, 0x00, pos[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncDisablePower(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3004,0x00,0x00000000);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncEnablePower(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3004, 0x00, 0x00000001);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncOpenBreak(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3150,0x00,0x00000001);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncClearError(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	printf("dunkermotoren_api::syncClearError()\n");
	VSCAN_MSG msgs[2*node_count];
	int count;
	
	//3350.00 096Ah
//3900.00 1
//3962.00 200
	for(count=0;count<node_count;count++)
	{
		msgs[(2*count)+0]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3000,0x00,0x00000001);
		msgs[(2*count)+1]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3350,0x00,0x096A);
		//msgs[(4*count)+2]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3900,0x00,1);
		//~ msgs[(2*count)+2]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3962,0x00,200);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count*2);
	return status;
}

int Dunker_syncSetOperationMode(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t mode)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(), 0x3003, 0x00, mode);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetVel(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* velocity)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3300, 0x00, velocity[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

//~ int Dunker_getVel(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* vel)
//~ {
	//~ 
//~ }

//~ int Dunker_syncSetDinamicLimitations(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
//~ {
	//~ VSCAN_MSG msgs[7*node_count];
	//~ int count;
	//~ for(count=0;count<node_count;count++)
	//~ {
		//~ //Current Dynamic limitation
		//~ //Curr Limit Pos
		//~ msgs[(7*count)+0] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3221, 0x00, 60000 );
		//~ //Curr Limit Neg
		//~ msgs[(7*count)+1] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3223, 0x00, 60000 );
		//~ //CurrDynLimitOff()
		//~ msgs[(7*count)+2] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000000);
		//~ //CurrDynLimitOn() - Dynamic Current limitation On
		//~ msgs[(7*count)+3] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x00, 0x00000001);
		//~ //CurrDynLimitPeak(mA) - Peak current
		//~ msgs[(7*count)+4] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x01, 60000);
		//~ //CurrDynLimitCont(mA) - Continous current
		//~ msgs[(7*count)+5] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x02, 10000);
		//~ //CurrDynLimitTime(ms) - Time for Peak current
		//~ msgs[(7*count)+6] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3224, 0x03, 2000);
	//~ }
	//~ int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 7*node_count);
	//~ return status;
//~ }

int Dunker_syncSetCurrentLimitPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3221, 0x00, limits[count] );
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetCurrentLimitNeg(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{	
		msgs[count] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3223, 0x00, limits[count] );
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

//POSITIVE AND NEGATIVE
int Dunker_syncSetCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits)
{
	printf("dunkermotoren_api::syncSetCurrentLimit(): \n");
	VSCAN_MSG msgs[2*node_count];
	int count;
	for(count=0;count<node_count;count++)
	{	
		msgs[(2*count)+0] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3223, 0x00, limits[count] );
		msgs[(2*count)+1] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3221, 0x00, limits[count] );
		printf("\tdunkermotoren_api::syncSetCurrentLimit(): for %d\n", count);
		Dunker_printMessageData(msgs[(2*count)+0]);
		Dunker_printMessageData(msgs[(2*count)+1]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, 2*node_count);
	return status;
}


int Dunker_syncConfigureErrorsAndFeedback(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	printf("dunkermotoren_api::syncConfigureErrorsAndFeedback():%d\n", node_count);
	VSCAN_MSG msgs[11*node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
	
		//#############################################################################
		//Primary Velocity and Position Controller
		//VelKp
		msgs[(11*count)+0] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3310, 0x00, 300);
		//VelKp_Stop
		msgs[(11*count)+1] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3310, 0x01, 50);
		
		msgs[(11*count)+2] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3310, 0x01, 60);
		//~ //VelKi
		msgs[(11*count)+3] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3311, 0x00, 0);
		//~ //VelKd
		msgs[(11*count)+4] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3312, 0x00, 0x00000000);
		//~ //VelKiLimit
		msgs[(11*count)+5] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3313, 0x00, 0x00000000);
//~ 
		//~ //Secondary Velocity Controller
		//~ //sVelKp
		msgs[(11*count)+6] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3510, 0x00, 100);
		//~ //sVelKi
		msgs[(11*count)+7] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3511, 0x00, 10);
//~ 
		//~ //Current Controller
		//~ //curr Kp
		msgs[(11*count)+8] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3210, 0x00, 100);
		//~ //curr Ki
		msgs[(11*count)+9] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3211, 0x00, 100);
		//~ //Feed forward***************************************************************
		//~ //Vel Feedforward
		msgs[(11*count)+10] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(),0x3314, 0x00, 1000);
	}
	//int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count*11);
	int status = 0;
	return status;
}

//set the maximum deviation between command and actual position
int Dunker_syncSetMaxPosFollowingError(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t max_err)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3732, 0x00, max_err);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetActualPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t* pos)
{
	printf("dunkermotoren_api::syncSetActualPosition()\n");
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(),0x3762, 0x00, pos[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	printf("dunkermotoren_api::syncSetActualPosition(): set\n");
	return status;
}


int Dunker_syncClearActualPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds)
{
	printf("dunkermotoren_api::syncClearActualPosition()\n");
	uint32_t poss[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		poss[count]=0;
	}
	int status = Dunker_syncSetActualPosition(Handle, node_count, NodeIds, poss);
	return status;
}


int Dunker_syncSetAcceleration(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* deltaV, int* deltaT)
{
	VSCAN_MSG msgs[node_count*2];
	int count;
	for(count=0;count<node_count;count++)
	{
		printf("dunkermotoren_api::Dunker_syncSetAcceleration() %d %d\n", deltaV[count], deltaT[count]);
		msgs[(2*count)+0] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3340, 0x00, deltaV[count]);
		Dunker_printMessageData(msgs[(2*count)+0]);
		msgs[(2*count)+1] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3341, 0x00, deltaT[count]);
		Dunker_printMessageData(msgs[(2*count)+1]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count*2);
	return status;
}
 
int Dunker_syncSetDecceleration(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* deltaV, int* deltaT)
{
	VSCAN_MSG msgs[node_count*2];
	int count;
	for(count=0;count<node_count;count++)
	{
		printf("dunkermotoren_api::Dunker_syncSetDecceleration() %d %d\n", deltaV[count], deltaT[count]);
		msgs[(2*count)+0] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3342, 0x00, deltaV[count]);
		Dunker_printMessageData(msgs[(2*count)+0]);
		msgs[(2*count)+1] = Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3343, 0x00, deltaT[count]);
		Dunker_printMessageData(msgs[(2*count)+0]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count*2);
	return status;
}


int Dunker_getStatusWord(VSCAN_HANDLE Handle, uint8_t NodeId, int* status_word)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3002, 0x00);
	//~ Dunker_printMessageData(msg);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	//~ Dunker_printMessageData(msg);
	memcpy(status_word,&msg.Data[4],4);
	//~ for(int x=0; x<32; x++)
	//~ {
		//~ printf("%d",!((*status_word>>x)%2==0));
	//~ }
	//~ printf("\n");
	return status;
}

int Dunker_syncGetStatusWord(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* status_words)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3002, 0x00);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		memcpy(&status_words[count], &msgs[count].Data[4],4);
	}
	return status;
}

int Dunker_getErrorRegister(VSCAN_HANDLE Handle, uint8_t NodeId, int* error_register)
{
	VSCAN_MSG msg;
	msg = Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3001, 0x00);
	//~ Dunker_printMessageData(msg);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	printf("getErrorRegister\n");
	Dunker_printMessageData(msg);
	memcpy(error_register,&msg.Data[4],4);
	return status;
}


#define BIT0 ((uint32_t)1)
#define BIT1 ((uint32_t)2)
#define BIT2 ((uint32_t)4)
#define BIT3 ((uint32_t)8)
#define BIT4 ((uint32_t)16)
#define BIT5 ((uint32_t)32)
#define BIT6 ((uint32_t)64)
#define BIT7 ((uint32_t)128)
#define BIT8 ((uint32_t)256)
#define BIT9 ((uint32_t)512)
#define BIT10 ((uint32_t)1024)
#define BIT11 ((uint32_t)2048)
#define BIT12 ((uint32_t)4096)
#define BIT13 ((uint32_t)8192)
#define BIT14 ((uint32_t)16384)
#define BIT15 ((uint32_t)32768)
#define BIT16 ((uint32_t)65536)
#define BIT17 ((uint32_t)131072)
#define BIT18 ((uint32_t)262144)
#define BIT19 ((uint32_t)524288)
#define BIT20 ((uint32_t)1048576)
#define BIT21 ((uint32_t)2097152)
#define BIT22 ((uint32_t)4194304)
#define BIT23 ((uint32_t)8388608)
#define BIT24 ((uint32_t)16777216)
#define BIT25 ((uint32_t)33554432)
#define BIT26 ((uint32_t)67108864)
#define BIT27 ((uint32_t)134217728)
#define BIT28 ((uint32_t)268435456)
#define BIT29 ((uint32_t)536870912)
#define BIT30 ((uint32_t)1073741824)
#define BIT31 ((uint32_t)2147483648)


void Dunker_printStatusWordInfo(uint16_t status_word)
{
	printf("dunkermotoren_api::printStatusWordInfo(): \n\t");
	qDebug()<<"\t"+QString("0000000000000000"+QString::number(status_word,2)).mid(QString::number(status_word,2).size());
	printf("\n");
	printf("\tController and power stage are enabled = %d\n", (status_word & BIT0)!=0);
	
	printf("\tAn error has occurred = %d\n", (status_word & BIT1)!=0);
	
	printf("\tNot used at the moment = %d\n", (status_word & BIT2)!=0);
	
	printf("\tMoving is active = %d\n", (status_word & BIT3)!=0);
	
	printf("\tTarget value was reached = %d\n", (status_word & BIT4)!=0);
	
	printf("\tTarget value was limited = %d\n", (status_word & BIT5)!=0);
	
	printf("\tFollowing error was exceeded = %d\n", (status_word & BIT6)!=0);
	
	printf("\tHoming was successful = %d\n", (status_word & BIT7)!=0);
	
	printf("\tPOS_Mova or POS_Movr was excepted = %d\n", (status_word & BIT8)!=0);
	
	printf("\tQuickstart command (DEV_Cmd2) was received = %d\n", (status_word & BIT9)!=0);	
	
	printf("\tQuickstart command (DEV_Cmd2) was not excepted = %d\n", (status_word & BIT10)!=0);
	
	printf("\tStop or Halt was executed = %d\n", (status_word & BIT11)!=0);
	
	printf("\tCurrent was limited = %d\n", (status_word & BIT12)!=0);
	
	printf("\tVelocity was limited = %d\n", (status_word & BIT13)!=0);
	
	printf("\tPosition was limited = %d\n", (status_word & BIT14)!=0);
	
	printf("\tSVelocity was limited. = %d\n", (status_word & BIT15)!=0);
}

bool Dunker_errorFlag(uint32_t status_word)
{
	return (status_word & BIT1)!=0;
}

int Dunker_setRampType(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t type)
{
	//set operation mode
	VSCAN_MSG msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x334C,0x00, type);
	printf("dunkermotoren_api::setRampType():  Setting ramp type: \n");
	Dunker_printMessageData(msg);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_syncSetRampType(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t type)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultWriteCommandSpecifier(), 0x334C, 0x00, type);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}


int Dunker_getGearSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int* revol)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3B19, 0x01);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*revol = Dunker_getReadedData(msg);
	return status;
}

int Dunker_getMotorSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int* revol)
{
	VSCAN_MSG msg;
	msg=Dunker_getMessageData(NodeId, Dunker_getDefaultReadCommandSpecifier(), 0x3B19, 0x00);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	*revol = Dunker_getReadedData(msg);
	return status;
}

int Dunker_setGearSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int revol)
{
	VSCAN_MSG msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3B19, 0x01, revol);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}

int Dunker_setMotorSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int revol)
{
	VSCAN_MSG msg = Dunker_getMessageData(NodeId,Dunker_getDefaultWriteCommandSpecifier(),0x3B19, 0x00, revol);
	int status = Dunker_writeWaitReadMessage(Handle, &msg);
	return status;
}


int Dunker_syncGetPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* positions)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3762, 0x00);
//		printf("position %d\n",msgs[0]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		positions[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_syncGetVelocity(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* velocities)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3A04, 0x01);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		velocities[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_syncGetMotorVoltages(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* volts)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3111, 0x00);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		volts[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_syncSetVelKps(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kps)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3310, 0x00, kps[count]);
		printf("KPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPSKPS");
		Dunker_printMessageData(msgs[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetVelKis(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kis)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3311, 0x00, kis[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncSetVelKds(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3312, 0x00, kds[count]);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	return status;
}

int Dunker_syncGetVelKps(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kps)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3310, 0x00);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		kps[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_syncGetVelKis(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kis)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3311, 0x00);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		kis[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_syncGetVelKds(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kds)
{
	VSCAN_MSG msgs[node_count];
	int count;
	for(count=0;count<node_count;count++)
	{
		msgs[count]=Dunker_getMessageData(NodeIds[count],Dunker_getDefaultReadCommandSpecifier(),0x3312, 0x00);
	}
	int status = Dunker_multiWriteWaitReadMessage(Handle,msgs, node_count);
	for(count=0;count<node_count;count++)
	{
		kds[count] = Dunker_getReadedData(msgs[count]);
	}
	return status;
}

int Dunker_JoseMateos(VSCAN_HANDLE Handle)
{
	VSCAN_MSG msgs[5];
	/*int count;
	for(count=0;count<node_count;count++)
	{
		//msgs[(11*count)+1]=Dunker_getMessageData(NodeIds[count], Dunker_getDefaultWriteCommandSpecifier(), 0x3311, 0x00, kis[count]);
		
	}*/

	int ID = 0x7e5;
	int Size2 = 2;
	int Size3 = 3;
	msgs[0].Flags = VSCAN_FLAGS_STANDARD;
	msgs[0].Id = ID;
	msgs[0].Size = Size2;
	msgs[0].Data[0] = 0x04;
	msgs[0].Data[1] = 0x01;
	msgs[0].Data[2] = 0x00;
	msgs[0].Data[3] = 0x00;
	msgs[0].Data[4] = 0x00;
	msgs[0].Data[5] = 0x00;
	msgs[0].Data[6] = 0x00;
	msgs[0].Data[7] = 0x00;


	msgs[1].Flags = VSCAN_FLAGS_STANDARD;
	msgs[1].Id = ID;
	msgs[1].Size = Size3;
	msgs[1].Data[0] = 0x13;
	msgs[1].Data[1] = 0x00;
	msgs[1].Data[2] = 0x04;
	msgs[1].Data[3] = 0x00;
	msgs[1].Data[4] = 0x00;
	msgs[1].Data[5] = 0x00;
	msgs[1].Data[6] = 0x00;
	msgs[1].Data[7] = 0x00;

	msgs[2].Flags = VSCAN_FLAGS_STANDARD;
	msgs[2].Id = ID;
	msgs[2].Size = Size3;
	msgs[2].Data[0] = 0x15;
	msgs[2].Data[1] = 0xAA;
	msgs[2].Data[2] = 0xAA;
	msgs[2].Data[3] = 0x00;
	msgs[2].Data[4] = 0x00;
	msgs[2].Data[5] = 0x00;
	msgs[2].Data[6] = 0x00;
	msgs[2].Data[7] = 0x00;

	msgs[3].Flags = VSCAN_FLAGS_STANDARD;
	msgs[3].Id = ID;
	msgs[3].Size = Size2;
	msgs[3].Data[0] = 0x04;
	msgs[3].Data[1] = 0x00;
	msgs[3].Data[2] = 0x00;
	msgs[3].Data[3] = 0x00;
	msgs[3].Data[4] = 0x00;
	msgs[3].Data[5] = 0x00;
	msgs[3].Data[6] = 0x00;
	msgs[3].Data[7] = 0x00;

	msgs[4].Flags = VSCAN_FLAGS_STANDARD;
	msgs[4].Id = ID;
	msgs[4].Size = Size2;
	msgs[4].Data[0] = 0x04;
	msgs[4].Data[1] = 0x00;
	msgs[4].Data[2] = 0x00;
	msgs[4].Data[3] = 0x00;
	msgs[4].Data[4] = 0x00;
	msgs[4].Data[5] = 0x00;
	msgs[4].Data[6] = 0x00;
	msgs[4].Data[7] = 0x00;

	int status = Dunker_multiWriteWaitReadMessage(Handle, msgs, 5);
	return status;
}


