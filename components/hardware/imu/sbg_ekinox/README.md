# sbg_ekinox

instala libreria https://github.com/SBG-Systems/sbgECom
- ubicamos en la carpera de software
- descargamos repo con  ``git add https://github.com/SBG-Systems/sbgECom.git``
- compilamos con ```cmake -Bbuild -DBUILD_EXAMPLES=ON -DBUILD_TOOLS=ON && cmake --build build```
- instalamos con ``sudo make install``

en el caso de que no funcione con el ```sudo make install``` existe otra opcion mas chapuza:

en el componente realiza el siguiente comando ``mkdir lib``
una vez compilado mete el build/libsbgECom.a del repo en lib/ del componente
realiza los siguente comando ``mkdir include && find src -name "*.h" -exec cp --parents \{} include/ \; && find common -name "*.h" -exec cp --parents \{} include/ \;``
este genera una carpeta include con todos los .h necesarios, copiamos esta carpeta include en al raiz del componente
introducir en en CMakeList.txt las lineas
```
link_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include/src ${CMAKE_SOURCE_DIR}/include/common)
```

Intro to component here



## EXPLANSION CODE 
Si se quieren introducir nuevos datos se usara la clase `SbgBinaryLogData` entro de la funcion de callback esta esta compuesto por los siguiente parametros a 23/08/2023.

La comunicacion entre el main y el calback se realizara mediante el parametro de la funcion `pUserArg` siendo este un `*void`, siendo necesario un casteo al tipo de dato deseado

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


## Configuration parameters
As any other component, *sbg_ekinox* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
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
