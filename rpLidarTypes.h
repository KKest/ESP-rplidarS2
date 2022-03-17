/*
 *  @author KKest
 *		@created 19.01.2022
 *	
 * Types for rpLidar library
 *
 */

#ifndef rplidarTypes_h
#define rplidarTypes_h

#include "Arduino.h"

typedef uint8_t rp_descriptor_t[7];
typedef uint8_t rq_Packet_t[9];
typedef uint8_t rq_message_t[2];


/// hold a measure point for standard scan mode
typedef struct scanDataPoint
{
	uint8_t quality;
	uint8_t angle_low;
	uint8_t angle_high;
	uint8_t distance_low;
	uint8_t distance_high;
}stScanDataPoint_t;

typedef struct Point
{
	double angle; 
	uint16_t distance; 
}point_t;

typedef struct deviceHealtStatus
{
	uint8_t status;
	uint8_t errorCode_low;
	uint8_t errorCode_high;
}stDeviceStatus_t;


typedef struct expressData
{
	uint16_t angle;
	uint16_t cabin[40];
}stExpressDataPacket_t;

typedef struct expressDataStorage
{
	uint8_t angle_low;
	uint8_t angle_high;
	uint16_t distance;
}expressDataStorage_t;

typedef struct rp_stDeviceInfo
{
  uint8_t model;
  uint8_t firmware_minor;
  uint8_t firmware_major;
  uint8_t hardware;
  uint8_t serialnumber[16];
}stDeviceInfo_t;

enum enDescriptor
{
		legacyVersion,  ///< Legacy scan version
		extendedVersion, ///< Extendet scan version
		denseVersion,	 ///< Dense scan version
		startScan,		 ///< start scan
		forceScan,		 ///< force to scan in idle mode
		deviceInfo,		 ///< deviceInfo of the Lidar
		healthInfo,		 ///< Error Codes and Status
		sampleRate,		 ///< momentary sampleRate
		deviceConf		 
};

enum enRequest
{
	rq_stop,
	rq_reset,
	rq_scan,
	rq_scanExpress,
	rq_scanForce,
	rq_info,
	rq_health,
	rq_sampleRate,
	rq_deviceConf
};

enum enMode
{
	stop,
	standard,
	express
};

extern rp_descriptor_t resp_descriptor[];	///< List of Response descriptors 
extern rq_message_t req_message[];			///< List of Request Messages
extern rq_Packet_t req_Express[];			///< Request Message for Express Scan Modes
#endif
