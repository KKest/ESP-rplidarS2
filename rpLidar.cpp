/*
 *  @author KKest
 *		@created 10.01.2022
 *
 * Library to control an rpLidar S2
 *
 */
#include "rpLidar.h"
#include "Arduino.h"


rpLidar::rpLidar(HardwareSerial *_mySerial,uint32_t baud)
{
	serial=_mySerial;
	serial->begin(baud);
}



stDeviceInfo_t rpLidar::getDeviceInfo()
{
	clearSerialBuffer();
	stDeviceInfo_t info;
	rp_descriptor_t descr;
	serial->write((uint8_t*)&req_message[rq_info],2); //send Device Info request
	if(!checkForTimeout(10,27))	//wait for Response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&info,20);
	}
	return info;
}

stDeviceStatus_t rpLidar::getDeviceHealth()
{
	clearSerialBuffer(); //remove old data in SerialBuffer
	rp_descriptor_t descr;
	stDeviceStatus_t deviceStatus;
	serial->write((uint8_t*)&req_message[rq_health],2); //send device health request
	if(!checkForTimeout(400,10)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&deviceStatus,3);
	}
	return deviceStatus;
}

uint16_t rpLidar::scanExpress()
{
	start(express);
	return readMeasurePoints();
}

uint16_t rpLidar::scanStandard()
{
	start(standard);
	return readMeasurePoints();
}

void rpLidar::resetDevice()
{
	serial->write((uint8_t*)&req_message[rq_reset],2); //send reset request
	delay(800); //wait for reboot
	clearSerialBuffer(); //remove old data in SerialBuffer
	status=false;
}

void rpLidar::stopDevice()
{
	serial->write((uint8_t*)&req_message[rq_stop],2);
}

bool rpLidar::start(uint8_t _mode)
{
	resetDevice();
	clearSerialBuffer();
	switch(express)
	{
		case standard:
			serial->write((uint8_t*)&req_message[rq_scan],2); //standard scan request
			break;
		case express:
			serial->write((uint8_t*)&req_Express[legacyVersion],9); //express scan request
			break;
		default:
			return false;
			break;
	}

	rp_descriptor_t descr;

	if(!checkForTimeout(100,7)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
		switch(_mode)
		{
			case standard:
				scanMode=_mode;
				status=true;
				return compareDescriptor(descr,resp_descriptor[startScan]);
				break;

			case express:
				scanMode=_mode;
				status=true;
				return compareDescriptor(descr,resp_descriptor[legacyVersion]);
				break;
			default :
				Serial.print("Kein Mode : ");
				Serial.println(_mode);
				scanMode=stop;
				status=false;
				return false;
				break;
		}

	}
	return false;
}

uint16_t rpLidar::readMeasurePoints()
{
	uint16_t count=0;
	switch(scanMode)
	{
		case standard:
			count=awaitStandardScan();
			break;
		case express:
			count=awaitExpressScan();
			break;
	}
	return count;
}


uint16_t rpLidar::awaitStandardScan()
{
	uint8_t *pBuff=(uint8_t*)&DataBuffer; //Pointer to Buffer
	uint16_t count=0;
	stScanDataPoint_t point;
	bool frameStart=false;

	uint32_t startTime=millis();
	while(millis()<(startTime+5000)) //timeout after 5 seconds
	{
		if(serial->available()>=5)
		{
			serial->readBytes((uint8_t*)&point,5);

			//search for frameStart
				// new Frame? S=1 !S=0 an checkbit information can be found in protocol datasheet
			if((point.quality&0x01)&&(!(point.quality&0x02))&&!frameStart) //  framestart? S=1 !S=0
			{
				if(point.angle_high&0x01) //check Bit
				{
					frameStart=true;
				}
			}
			else if(frameStart&&(point.quality&0x01)&&!(point.quality&0x02)&&count>1) //2. framestart?
			{
				if(point.angle_high&0x01)
				{
					return count;
				}
			}
			else if(frameStart)
			{
				memmove(pBuff,(uint8_t*)&point,sizeof(point)); //copy memory from incoming buffer to DataBuffer
				count++; //new point
				if(count<sizeof(DataBuffer)/sizeof(scanDataPoint))//inside the array bondaries?
				{
					pBuff=pBuff+5; //move pointer to next measure point in storage
				}
			}
		}
	}
	return count;
}

uint16_t rpLidar::awaitExpressScan()
{
	uint8_t Buffer[2];
	uint16_t count=0; //count of Packets with angle and 40 cabins
	uint8_t cabinCount=0; //count of cabin which haves to be written
	serial->flush();
	uint8_t crc=0;
	uint32_t internTimeCount=millis();
	while(count<79)
	{
		if(serial->available()>=1)
		{
			uint8_t sync1=serial->read(); //Sync byte 1 of Packet
			if(((sync1&0xF0)==0xA0))
			{
				while(serial->available()<=1)
				{
						if(millis()>internTimeCount+500)
						{
							return 0;
						}
					
				}
				internTimeCount=millis();
				uint8_t sync2=serial->read(); //Sync byte 2 of Packet
				if((sync2&0xF0)==0x50)
				{
					crc=(sync2<<4)|(sync1&0x0F);

					while(serial->available()<2)
					{
						if(millis()internTimeCount+500)return 0;
					}
					serial->readBytes((uint8_t*)&Buffer,2);//read angle
					ExpressDataBuffer[count].angle=(Buffer[1]<<8)|Buffer[0]; //connect angle low and high byte
					while(cabinCount<40)
					{
						while(serial->available()<2)
						{
							if(millis()internTimeCount+500)return 0;
						}
						if(serial->available()>=2)//cabin available?
						{
							serial->readBytes((uint8_t*)&Buffer,2);
							ExpressDataBuffer[count].cabin[cabinCount]=Buffer[1]<<8|Buffer[0];
							cabinCount++;
							internTimeCount=millis();
						}
					}
					cabinCount=0;
					if(!checkCRC(ExpressDataBuffer[count],crc))
					{
						return 0;
					}
					count++;
				}
			}
		}
	}
	ExpressDataToPointArray((stExpressDataPacket_t*) ExpressDataBuffer, count-1);
	return count;
}


void rpLidar::setAngleOfInterest(uint16_t _left,uint16_t _right)
{
	//setter
	interestAngleLeft=_left;
	interestAngleRight=_right;
}


bool rpLidar::isDataBetweenBorders(stScanDataPoint_t _point)
{
	float angle=calcAngle(_point.angle_low,_point.angle_high);
	if((angle>=interestAngleLeft)&&(angle<=interestAngleRight))
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataBetweenBorders(float _angle)
{
	if((_angle>interestAngleLeft)&&(_angle<interestAngleRight))
	{
		return true;
	}
	return false;
}


bool rpLidar::isDataValid(stScanDataPoint_t _point)
{
	if(calcDistance(_point.distance_low,_point.distance_high)>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataValid(uint16_t _distance)
{
	if(_distance>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isRunning()
{
	return status;
}

uint8_t rpLidar::isScanMode()
{
	return scanMode;
}

float rpLidar::calcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t winkel=_highByte<<7;
	winkel|=_lowByte>>1;
	return winkel/64.0;
}

float rpLidar::calcCapsuledAngle(uint16_t _Wi,uint16_t _Wi2,uint8_t _k)
{
	float angle1=_Wi/64.00;
	float angle2=_Wi2/64.00;
	float result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;
}




float rpLidar::calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance=(_highByte)<<8;
	distance|=_lowByte;
	return distance/4.0;
}



//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											 Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//

void rpLidar::DebugPrintMeasurePoints(int16_t _count)
{
	Serial.println(_count-1);
	if(_count<=0)return;

    for(uint16_t i=0;i<_count-1;i++)
    {
		if(isDataBetweenBorders(DataBuffer[i])&&isDataValid(DataBuffer[i]))
		{
			Serial.print(i);
			Serial.print("\t|\t");
			Serial.print((DataBuffer[i].quality)>>2,DEC);
			Serial.print("\t|\t");
			Serial.print(calcAngle(DataBuffer[i].angle_low,DataBuffer[i].angle_high));
			Serial.print("\t|\t");
			Serial.print(calcDistance(DataBuffer[i].distance_low,DataBuffer[i].distance_high));
			Serial.println();
		}
	}
}

void rpLidar::DebugPrintDeviceErrorStatus(stDeviceStatus_t _status)
{
	Serial.println("\n--Device Health--");
	Serial.print("Status:");
	Serial.println(_status.status);
	Serial.print("Error Low:");
	Serial.print(_status.errorCode_low);
	Serial.print("Error High:");
	Serial.println(_status.errorCode_high);
	Serial.println('\n');
}

void rpLidar::DebugPrintDeviceInfo(stDeviceInfo_t _info)
{
	Serial.println("\n--Device Info--");
	Serial.print("Model:");
	Serial.println(_info.model);
	Serial.print("Firmware:");
	Serial.print(_info.firmware_major);
	Serial.print(".");
	Serial.println(_info.firmware_minor);
	Serial.print("Hardware:");
	Serial.println(_info.hardware);
	Serial.print("Serial Number:");
	for(uint16_t i=0;i<16;i++)
	{
		Serial.print(_info.serialnumber[i],HEX);
	}
	Serial.println('\n');

}

void rpLidar::DebugPrintDescriptor(rp_descriptor_t _descriptor)
{
	Serial.print ("Descriptor : ");
	for(uint8_t i=0;i<7;i++)
	{
		Serial.print(_descriptor[i],HEX);
		Serial.print("|");
	}
	Serial.println();
}

void rpLidar::DebugPrintBufferAsHex()
{
	for(uint16_t i=0;i<sizeof(DataBuffer)/sizeof(scanDataPoint);i++)
	{
		Serial.print("0x");
		Serial.print(DataBuffer[i].quality,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_low,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_high,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_low,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_high,HEX);
		Serial.println(",");
	}
	Serial.println();
}


//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											End Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//




bool rpLidar::compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2)
{
	for(size_t i=0;i<sizeof(rp_descriptor_t);i++)
	{
		if(_descr1[i]!=_descr2[i])
		{
			return false;
		}
	}
	return true;
}

void rpLidar::clearSerialBuffer()
{
	while(serial->available())//read as long the hardware buffer is not empty
	{
		serial->read();
	}
}

bool rpLidar::checkCRC(stExpressDataPacket_t _package,uint8_t _crc)
{
	uint8_t crc=0;
	crc=(uint8_t)_package.angle&0x00FF;
	crc^=(uint8_t)(_package.angle>>8);
	for(int i=0;i<40;i++)
	{
		crc^=(uint8_t)_package.cabin[i];
		crc^=(uint8_t)(_package.cabin[i]>>8);

	}
	if(_crc==crc)
	{
		return true;
	}
	return false;
}

bool rpLidar::checkForTimeout(uint32_t _time,size_t _size)
{
	float startTime=millis();
	while(!(serial->available()>=_size))
	{
		if(millis()>(startTime+_time)){
			Serial.println("Lidar Timeout");
			return true;
		}
	}
	return false;
}

bool rpLidar::ExpressDataToPointArray(stExpressDataPacket_t* _packets, uint16_t _count)
{
	uint16_t index=0;
	/// do a copy of expressData packet buffer and replace the angle of each cabin with the real angle
	for(uint16_t i=0;i<_count-2;i++) //each expressData packet
	{
		for(uint16_t j=0;j<40;j++) //each cabin in expressData packet
		{
			double angle=calcAngle(_packets,j); //calculate the angle of current cabin
			if(isDataBetweenBorders(angle)) //data correct interesting and in fov?
			{
				Data[index].angle=angle;
				Data[index].distance=_packets->cabin[j];
				index++;
			}
			else if(isDataBetweenBorders(angle))
			{
				Data[index].angle=0;
				Data[index].distance=0;
				index++;
			}
		}
		_packets++;
	}

/// sorting start
///Aufsteigend sortieren, höchster winkel am Ende des Arrays
	uint16_t tmpDistance; //temporary storage for value
	double tmpAngle; //temporary storage for value
	bool changed;
    int counter = index-1;

	do //sorts the data from low to high angle , highest angle is at the last index of the array
	{
		changed = false;
		for (int i = 0; i < counter; i++)
		{
			if (Data[i].angle > Data[i + 1].angle)
			{
				tmpAngle = Data[i + 1].angle;
				tmpDistance = Data[i + 1].distance;
				Data[i + 1].angle = Data[i].angle;
				Data[i + 1].distance = Data[i].distance;
				Data[i].angle = tmpAngle;
				Data[i].distance=tmpDistance;
				changed = true;
			}
		}
	}while(changed);
///sorting END
	return true;
}

double  rpLidar::calcAngle(stExpressDataPacket_t* _packets,uint16_t _k)
{
	double  angle1=(_packets->angle&0x7FFF)/64.00;
	_packets++;
	double  angle2=(_packets->angle&0x7FFF)/64.00;
	double  result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;

}