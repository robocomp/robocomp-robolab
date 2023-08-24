/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;

	// Close the sbgEcom library
	sbgEComClose(&comHandle);
	sbgInterfaceDestroy(&sbgInterface);
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;

	try
	{
		std::string IP_address = params.at("IP_address").value;
		int input_port = std::stoi(params.at("input_port").value);
		int output_port =  std::stoi(params.at("output_port").value);
		std::string rs232 = params.at("rs232").value;
		int baudrate =  std::stoi(params.at("baudrate").value);


		if (IP_address.size() == 0){
		//
		// Create a serial interface to communicate with the PULSE
		//
		errorCode = sbgInterfaceSerialC//
	SbgInterface sbgInterface;
	SbgEComHandle comHandle;reate(&this->sbgInterface, rs232.c_str(), baudrate);
	}
	else{
		//
		// Create a serial interface to communicate with the PULSE
		//
		errorCode = sbgInterfaceUdpCreate(&this->sbgInterface, sbgNetworkIpFromString(IP_address.c_str()), input_port, output_port);
	}
	
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }

	if (errorCode == SBG_NO_ERROR)
		{
			comHandle = init_sbg_ekinox(&sbgInterface, &this->data_imu);
		}
		else
		{
			SBG_LOG_ERROR(errorCode, "unable to open serial interface");
			exit(-1);
		}

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = 5;// max 200Hz 

	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	// Try to read a frame
	SbgErrorCode errorCode = sbgEComHandle(&comHandle);
	if (errorCode != SBG_NOT_READY)
		SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");
	#if DEBUG
	else
		show_data(&data_imu);
	#endif
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
/*!
 *	Callback definition called each time a new log is received.
 * 
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	msgClass								Class of the message we have received
 *	\param[in]	msg										Message ID of the log received.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode SpecificWorker::callback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
	RoboCompIMU::DataImu *data = (RoboCompIMU::DataImu *)pUserArg;
    assert(pLogData);

	SBG_UNUSED_PARAMETER(pHandle);
	SBG_UNUSED_PARAMETER(pUserArg);

	/////////ALL DATA IN SbgBinaryLogData/////////
	if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0) {
		// Handle separately each received data according to the log ID
		switch (msg) {
			case SBG_ECOM_LOG_EKF_EULER:// more data in SBG_ECOM_LOG_EKF_EULER
				#if DEBUG
				printf("Euler status: %u *********Time: %u\n", pLogData->ekfEulerData.status, pLogData->ekfEulerData.timeStamp);
				#endif
				data->rot.Roll = pLogData->ekfEulerData.euler[0];
				data->rot.Pitch = pLogData->ekfEulerData.euler[1];
				data->rot.Yaw = pLogData->ekfEulerData.euler[2];
				break;
			case SBG_ECOM_LOG_IMU_DATA:// more data in SBG_ECOM_LOG_IMU_DATA
				#if DEBUG
				printf("IMU status: %u *********Time: %u\n", pLogData->fastImuData.status, pLogData->fastImuData.timeStamp);
				#endif
				data->acc.XAcc = pLogData->fastImuData.accelerometers[0];
				data->acc.YAcc = pLogData->fastImuData.accelerometers[1];
				data->acc.ZAcc = pLogData->fastImuData.accelerometers[2];
				data->gyro.XGyr = pLogData->fastImuData.gyroscopes[0];
				data->gyro.YGyr = pLogData->fastImuData.gyroscopes[1];
				data->gyro.ZGyr = pLogData->fastImuData.gyroscopes[2];
			case SBG_ECOM_LOG_MAG:// more data in SBG_ECOM_LOG_MAG
				#if DEBUG
				printf("Magnetic status: %u *********Time: %u\n", pLogData->ekfEulerData.status, pLogData->ekfEulerData.timeStamp);
				#endif
				data->mag.XMag = pLogData->magData.magnetometers[0];
				data->mag.YMag = pLogData->magData.magnetometers[1];
				data->mag.ZMag = pLogData->magData.magnetometers[2];
				break;
			case SBG_ECOM_LOG_EKF_NAV:// more data in SbgLogEkfNavData
				#if NAVDATA
				printf("Naval status: %u *********Time: %u\n", pLogData->ekfNavData.status,  pLogData->ekfNavData.timeStamp);
				printf("Velocity:  %fx, %fy,%fz\n Position: %fx, %fy, %fz\n, undulation: %f", pLogData->ekfNavData.velocity[0], 
				pLogData->ekfNavData.velocity[1], pLogData->ekfNavData.velocity[2], pLogData->ekfNavData.position[0], 
				pLogData->ekfNavData.position[1], pLogData->ekfNavData.position[2], pLogData->ekfNavData.undulation);
				#endif
				break;
			case SBG_ECOM_LOG_GPS1_VEL:// more data in SbgLogGpsVel
				#if GPSDATA
				printf("Naval velocity status: %u *********Time: %u\n", pLogData->gpsVelData.status,  pLogData->gpsVelData.timeStamp);
				printf("Velocity:  %fx, %fy, %fz", pLogData->gpsVelData.velocity[0], 
				pLogData->gpsVelData.velocity[1], pLogData->gpsVelData.velocity[2]);
				#endif
				break;
			case SBG_ECOM_LOG_GPS1_POS:// more data in SbgLogGpsPos
				#if GPSDATA
				printf("Naval position status: %u *********Time: %u\n", pLogData->gpsPosData.status,  pLogData->gpsPosData.timeStamp);
				printf("Latitude:  %f,  Longitude: %f, Altitude: %f", pLogData->gpsPosData.latitude, 
				pLogData->gpsPosData.longitude, pLogData->gpsPosData.altitude);
				#endif
				break;
			case SBG_ECOM_LOG_GPS1_RAW:
				#ifdef GPSDATA
				;//RAW DATA
				#endif
				break;
			case SBG_ECOM_LOG_GPS1_HDT:// more data in SbgLogGpsHdt
				#if GPSDATA
				printf("Naval position status: %u *********Time: %u\n", pLogData->gpsHdtData.status,  pLogData->gpsHdtData.timeStamp);
				printf("Heading:  %f,  Pitch: %f", pLogData->gpsHdtData.heading, 
				pLogData->gpsHdtData.pitch);
				#endif
				break;
			case SBG_ECOM_LOG_UTC_TIME:// more data in SbgLogUtcData
				break;	
			case SBG_ECOM_LOG_STATUS:// more data in SbgLogStatusData
				break;
			default:
				printf("an unread callback: %u\n", msg);
				break;
		}
	}
	return SBG_NO_ERROR;
}

/*!
 * Get and print product info.
 *
 * \param[in]	pECom					SbgECom instance.
 * \return								SBG_NO_ERROR if successful.
 */
SbgErrorCode  SpecificWorker::print_product_info(SbgEComHandle *pECom)
{
	SbgErrorCode					errorCode;
	SbgEComDeviceInfo				deviceInfo;
	assert(pECom);

	// Get device inforamtions
	errorCode = sbgEComCmdGetInfo(pECom, &deviceInfo);

	// Display device information if no error
	if (errorCode == SBG_NO_ERROR)
	{
		char	calibVersionStr[32];
		char	hwRevisionStr[32];
		char	fmwVersionStr[32];		

		sbgVersionToStringEncoded(deviceInfo.calibationRev, calibVersionStr, sizeof(calibVersionStr));
		sbgVersionToStringEncoded(deviceInfo.hardwareRev, hwRevisionStr, sizeof(hwRevisionStr));
		sbgVersionToStringEncoded(deviceInfo.firmwareRev, fmwVersionStr, sizeof(fmwVersionStr));

		printf("Serial Number: %d\n",	deviceInfo.serialNumber);
		printf("Product Code: %s\n", deviceInfo.productCode);
		printf("Hardware Revision: %s\n", hwRevisionStr);
		printf("Firmware Version: %s\n", fmwVersionStr);
		printf("Calib. Version: %s\n", calibVersionStr);
		printf("\n");
	}
	else
	{
		SBG_LOG_WARNING(errorCode, "Unable to retrieve device information");
		exit(-1);
	}
	return errorCode;
}

/*!
 * Execute the ellipseMinimal example given an opened and valid interface.
 * 
 * \param[in]	pInterface							Interface used to communicate with the device.
 * \param  data_imu 								Callback data
 * \return											SBG_NO_ERROR if successful.
 */
SbgEComHandle  SpecificWorker::init_sbg_ekinox(SbgInterface *pInterface, RoboCompIMU::DataImu *data_imu )
{
	SbgErrorCode			errorCode = SBG_NO_ERROR;
	SbgEComHandle			comHandle;
	assert(pInterface);

	// Create the sbgECom library and associate it with the created interfaces
	errorCode = sbgEComInit(&comHandle, pInterface);

	// Test that the sbgECom has been initialized
	if (errorCode == SBG_NO_ERROR)
	{
		printf("sbgECom version %s\n\n", SBG_E_COM_VERSION_STR);
		// Query and display produce info, don't stop if there is an error
		print_product_info(&comHandle);
		// Define callbacks for received data and display header
		sbgEComSetReceiveLogCallback(&comHandle, callback, data_imu);
	}
	else
	{
		SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
		exit(-1);
	}
	return comHandle;
}

void SpecificWorker::show_data(RoboCompIMU::DataImu *_data_imu)
{
	printf("Aceleration : %f, %f, %f\n",	_data_imu->acc.XAcc, _data_imu->acc.YAcc, _data_imu->acc.ZAcc);
	printf("Gyroscope : %f, %f, %f\n",	_data_imu->gyro.XGyr, _data_imu->gyro.YGyr, _data_imu->gyro.ZGyr);
	printf("Magnetic : %f, %f, %f\n",	_data_imu->mag.XMag, _data_imu->mag.YMag, _data_imu->mag.ZMag);
	printf("Orientation : %f, %f, %f\n",	_data_imu->rot.Roll, _data_imu->rot.Pitch, _data_imu->rot.Yaw);
	printf("\n");
}

RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
	return this->data_imu.acc;
}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
	return data_imu.gyro;
}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
	return data_imu;
}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
	return data_imu.mag;
}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
	return data_imu.rot;
}

void SpecificWorker::IMU_resetImu()
{
//implementCODE
}

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

