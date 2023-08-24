# sbg_ekinox

Install the library: https://github.com/SBG-Systems/sbgECom
- Navigate to the software folder.
- Clone the repository using  ``git add https://github.com/SBG-Systems/sbgECom.git``
- Compile with ```cmake -Bbuild -DBUILD_EXAMPLES=ON -DBUILD_TOOLS=ON && cmake --build build```
- Install using ``sudo make install``

If the command ```sudo make install``` doesn't work, there's an alternative method:

- Inside the component, execute the following command: ``mkdir lib``
- After compiling, move the build/libsbgECom.a from the repository to the lib/ folder of the component
- Execute the following commands: ``mkdir include && find src -name "*.h" -exec cp --parents \{} include/ \; && find common -name "*.h" -exec cp --parents \{} include/ \;``
- This creates an "include" folder with all necessary .h files. Copy this "include" folder to the root of the component.
- Add the following lines to CMakeList.txt:
```
link_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include/src ${CMAKE_SOURCE_DIR}/include/common)
```

## EXPANSION CODE  
If you wish to introduce new data, use the  `SbgBinaryLogData` class within the callback function. As of 23/08/2023, it comprises the following parameters.
```
/*!
 *	Union used to store received logs data.
 */
typedef union _SbgBinaryLogData
{
	SbgLogStatusData				statusData;			/*!< Stores data for the SBG_ECOM_LOG_STATUS message. */
	SbgLogImuData					imuData;			/*!< Stores data for the SBG_ECOM_LOG_IMU_DATA message. */
	SbgLogImuShort					imuShort;			/*!< Stores data for the SBG_ECOM_LOG_IMU_SHORT message. */
	SbgLogEkfEulerData				ekfEulerData;		/*!< Stores data for the SBG_ECOM_LOG_EKF_EULER message. */
	SbgLogEkfQuatData				ekfQuatData;		/*!< Stores data for the SBG_ECOM_LOG_EKF_QUAT message. */
	SbgLogEkfNavData				ekfNavData;			/*!< Stores data for the SBG_ECOM_LOG_EKF_NAV message. */
	SbgLogShipMotionData			shipMotionData;		/*!< Stores data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message. */
	SbgLogOdometerData				odometerData;		/*!< Stores data for the SBG_ECOM_LOG_ODO_VEL message. */
	SbgLogUtcData					utcData;			/*!< Stores data for the SBG_ECOM_LOG_UTC_TIME message. */
	SbgLogGpsPos					gpsPosData;			/*!< Stores data for the SBG_ECOM_LOG_GPS_POS message. */
	SbgLogGpsVel					gpsVelData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_VEL message. */
	SbgLogGpsHdt					gpsHdtData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_HDT message. */
	SbgLogRawData					gpsRawData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_RAW message. */
	SbgLogRawData					rtcmRawData;		/*!< Stores data for the SBG_ECOM_LOG_RTCM_RAW message. */
	SbgLogMag						magData;			/*!< Stores data for the SBG_ECOM_LOG_MAG message. */
	SbgLogMagCalib					magCalibData;		/*!< Stores data for the SBG_ECOM_LOG_MAG_CALIB message. */
	SbgLogDvlData					dvlData;			/*!< Stores data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK message. */
	SbgLogAirData					airData;			/*!< Stores data for the SBG_ECOM_LOG_AIR_DATA message. */
	SbgLogUsblData					usblData;			/*!< Stores data for the SBG_ECOM_LOG_USBL message. */
	SbgLogDepth						depthData;			/*!< Stores data for the SBG_ECOM_LOG_DEPTH message */
	SbgLogEvent						eventMarker;		/*!< Stores data for the SBG_ECOM_LOG_EVENT_# message. */
	SbgLogDiagData					diagData;			/*!< Stores data for the SBG_ECOM_LOG_DIAG message. */
	SbgLogSatGroupData				satGroupData;		/*!< Stores data for the SBG_ECOM_LOG_SAT message. */

	/* Fast logs */
	SbgLogFastImuData				fastImuData;		/*!< Stores Fast Imu Data for 1KHz output */

} SbgBinaryLogData;
```
Communication between the main function and the callback is done via the `pUserArg` function parameter. It is a `*void`, so casting to the desired data type is necessary.

## CONFIGURATION OF SBG EKINOX
Using a web browser, you can access the device by entering its IP address or by using the URL based on its serial number: `http://ekinox_023000133.local./`. In this case, "023000133" is the serial number.

## Configuration parameters
As any other component, *sbg_ekinox* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
CommonBehavior.Endpoints=tcp -p 10000

# Endpoints for implements interfaces
IMU.Endpoints=tcp -p 0

#Priority with ethernet, to use serial, leave IP blank.
IP_address = 192.168.50.50
input_port = 5678
output_port = 1234

rs232 = /dev/ttyUSB0
baudrate = 230400


InnerModelPath = innermodel.xml

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.MessageSizeMax=20004800
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <sbg_ekinox's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/sbg_ekinox config
```
