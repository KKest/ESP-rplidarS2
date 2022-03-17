# ESP32_Arduino_RPLIDAR_S2


## Gerneral Information
 **This Library is not supporting all features of the sensor.**
 **Do read the below lines what's supported and tested!**
 
It can be used to interface between an RPLIDAR S2 and an ESP32.

### Supported and tested boards and frameworks
- ESP32 with Arduino IDE At the moment is only the ESP32 tested. Should be possible to use every microcontroller with serial port supported baud rate of 1M and enough storage. Other 2D 360° Lidars could also work if they have the same protocol as the S2 got.

### Whats implemented
Currently, there are the following functions implemented in the library 
- Start and Stop scanning 
- Standard Scan frame reads from sensor 
- Express Scan reads from sensor 
- read of sensor status 
- read of sensor information

That's it for now.

## Library usage
Create an instance of the library with the following parameters 
- Address to the serial port of the port where the sensor is connected to 
- uint32_t baud rate of 1000000 for the S2

### Read Sensorinformation
`getDeviceInfo()` gets the Device address and Software Information of the sensor and returns the struct stDeviceInfo_t with the following informations.

```C++
struct rp_stDeviceInfo
{
  uint8_t model;
  uint8_t firmware_minor;
  uint8_t firmware_major;
  uint8_t hardware;
  uint8_t serialnumber[16];
}stDeviceInfo_t;
```

### read sensor status
`getDeviceHealth()`
The Status is not mentioned in the datasheet. If status is 0 the sensor is good. If the status equals 2 it's not ready for run
```C++
typedef struct deviceHealtStatus
{
	uint8_t status;
	uint8_t errorCode_low;
	uint8_t errorCode_high;
}stDeviceStatus_t;
```
### start and stop scan
A scan of the Sensor can be started with `start(uint8_t _mode)`. You have to pass the scan mode which the sensor should run
```C++
enum enMode
{
	stop,
	standard,
	express
};
```
To stop the scan mode `stopDevice()` can be used.

### read scan data
The data of a scan can be read with `readMeasurePoints()` (after a scan is initiated with `startScan()`) the measure points are saved to a data buffer.
```C++ 
stScanDataPoint_t DataBuffer[1500]` //is used for standard mode
point_t Data[1540]` //is used for express mode
```
The struct point_t is shown below
```C++ 
typedef struct Point
{
	double angle; 
	uint16_t distance; 
}point_t;
```
The buffer can be accessed with your program. The express mode is in most cases the better option because of the higher angle resolution and faster scans. The amount of time for refreshing the buffer Data is about 100ms. Standard mode is much slower.

### Debug
There are some debug functions implemented to show data to the serial monitor of your Computer
```C++
	void DebugPrintMeasurePoints(int16_t _count);	///< prints Standard Data in normal Format in Serial Monitor
	void DebugPrintDeviceErrorStatus(stDeviceStatus_t _status); ///< prints Status of lidar in Serial Monitor
	void DebugPrintDeviceInfo(stDeviceInfo_t _info);	///< prints Device Info in Serial Monitor
	void DebugPrintDescriptor(rp_descriptor_t _descriptor); ///< prints descriptor in Serial Monitor
	void DebugPrintBufferAsHex();				///< prints Standard Data as Hex splitted with ","  in Serial Monitor
```

