/*
 *  @author KKest
 *		@created 10.01.2022
 *	
 * Library to control an rpLidar S2
 *
 */
 
#ifndef rplidar_h
#define rplidar_h

#include "rpLidarTypes.h"

class rpLidar{
	public:
	
	/**
	 * Construcor of Class
	 *
	 * @param pointer to used USART
	 * @param Baudrate
	 */
	rpLidar(HardwareSerial *_serial,uint32_t baud);

	/**
	 * Gets the device info from rpLidar
	 *
	 *	It hold´s the Model no. , firmware and software vers.
	 *	and the serialnumber
	 *
	 * @return requested Device Info 
	 */
	stDeviceInfo_t getDeviceInfo();

	/**
	 * Gets the Status from rpLidar
	 *	u8 Status
	 *	u16 Errorcode
	 *
	 * @return Status
	 */	
	stDeviceStatus_t getDeviceHealth();
	
	/**
	 * Resets the lidar 
	 *
	 * should be executed if an error happend
	 */	
	void resetDevice();
	
	/**
	 * Stops the lidar if its turning
	 * goes into idle state
	 */	
	void stopDevice();
	
	/**
	 * Starts the Lidar and its measurement system
	 * 
	 * @param modus to run the lidar
	 * @return true if mode started correctly, fals if not 
	 */	
	bool start(uint8_t _mode);
	
	/**
	 * should be excecuted as often as possible to read the data from USART
	 * 
	 * @return the number of data for the running mode (express *40, standard*1) 
	 */	
	uint16_t readMeasurePoints();

	/**
	 * Sets the Angle for isDataBetweenBorders()
	 *
	 */	
	void setAngleOfInterest(uint16_t _left,uint16_t right);


	//Debug Funktionen
	void DebugPrintMeasurePoints(int16_t _count);	///< prints Standard Data in normal Format in Serial Monitor
	void DebugPrintDeviceErrorStatus(stDeviceStatus_t _status); ///< prints Status of lidar in Serial Monitor
	void DebugPrintDeviceInfo(stDeviceInfo_t _info);	///< prints Device Info in Serial Monitor
	void DebugPrintDescriptor(rp_descriptor_t _descriptor); ///< prints descriptor in Serial Monitor
	void DebugPrintBufferAsHex();				///< prints Standard Data as Hex splitted with ","  in Serial Monitor

	
	point_t Data[3250]; ///< stores the raw scan data
	stScanDataPoint_t DataBuffer[1500];	///<Storage to save the Data of a Standard Scan
	
	private:
	
	stExpressDataPacket_t ExpressDataBuffer[79];	///<Storge to save the Data of an Express Scan
	uint16_t interestAngleLeft;		///< left border of needed angle 180-360°
	uint16_t interestAngleRight;	///< right border of needed angle 0-180°

	uint8_t scanMode=stop; 			///< contains the actual scan mode of lidar
	bool status=false; 				///< contains the actual status of lidar
	
	/**
	 * Compares two Response Descriptors 
	 * @returns true if equal 
	 */	
	bool compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2);
	
	/**
	 * 
	 * Should be the same as mySerial->flush()
	 * but worked not as expected at test time
	 */		
	void clearSerialBuffer();

	/**
	 * Checks the CRC of an Express Scan Packet
	 * with XOR accumulation of each data byte
	 */		
	bool checkCRC(stExpressDataPacket_t _package,uint8_t _crc);

	/**
	 * Checks if no Serial Data ist available in a given time
	 * 
	 * @param wait time ms 
	 * @param amount of expected bytes
	 * @return true if a timeout happend
	 */	
	bool checkForTimeout(uint32_t _time,size_t _size);
	
	
	/**
	 * Calculates the angle of each Cabin in an ExpressDataPacket
	 *
	 * @param pointer to the packet
	 * @param current cabin
	 * @return calculated angle
	 */	
	double  calcAngle(stExpressDataPacket_t* _packets,uint16_t _k);
	
	/**
	 * Copy the data to new memory and calculate the true angle of each distance
	 * 
	 * @param Pointer to lidar recceive express databuffer
	 * @param counts of packages
	 * @return false if it failed
	 */	
	bool ExpressDataToPointArray(stExpressDataPacket_t* _packets, uint16_t _count);
		
	/**
	 * Tries to read a new full cycle of Points
	 * 
	 * @return the number of points 
	 */	
	uint16_t awaitStandardScan();

	/**
	 * tries to read a new full cycle of Points
	 * 
	 * @return the number of cabins 
	 */	
	uint16_t awaitExpressScan();
	
	/**
	 * Starts an express scan directly 
	 *
	 * @return the count of cabins with data (for each cabin 40 Data points)
	 */	
	uint16_t scanExpress();
	
	/**
	 * Starts an standard scan directly
	 *
	 * @return the count of measurement points
	 */	
	uint16_t scanStandard();
	
	/**
	 * Checks if the angle is between the wanted angle
	 * that`s set with setAngleOfInterest();
	 * 
	 * @param  Standard mode Point to check
	 * @return true if data is between borders
	 */	
	bool isDataBetweenBorders(stScanDataPoint_t _point);
	
	/**
	 * Checks if the angle is between the wanted angle
	 * that`s set with setAngleOfInterest();
	 * @overload
	 */	
	bool isDataBetweenBorders(float _angle);
	

	/**
	 * Calculates angle for Standard mode 
	 * According to the Datasheet for standard mode angle´s
	 * @param LS
	 * @param MS
	 * @returns angle 0.00-360.00
	 */	
	float calcAngle(uint8_t _lowByte, uint8_t _highByte);
	
	/**
	 * Calculates angle for Express mode 
	 * According to the Datasheet for express mode angle´s
	 * 
	 * @param Wi
	 * @param Wi+1
	 * @param k number of Cabin of Packet
	 * @returns angle 0.00-360.00
	 */	
	 float calcCapsuledAngle(uint16_t _angle1,uint16_t _angle2,uint8_t _k);
	
	/**
	 * Calculates the distance for Standard mode
	 * @returns distance in mm
	 */	
	 float calcDistance(uint8_t _lowByte, uint8_t _highByte);

	/**
	 * Checks thats the measured distance is valid
	 *
	 * @return true if its valid
	 */	
	bool isDataValid(stScanDataPoint_t _point);
	
	/**
	 * Checks thats the measured distance is valid
	 *
	 * @overload
	 */	
	bool isDataValid(uint16_t _distance);
	
	/**
	 * 
	 * @return true if the motor is running
	 */	
	bool isRunning();
	
	/**
	 * 
	 * @return actual running scanmode
	 */	
	uint8_t isScanMode(); 
	
	HardwareSerial *serial;		///< pointer to HardwareSerial USART 
};



#endif
