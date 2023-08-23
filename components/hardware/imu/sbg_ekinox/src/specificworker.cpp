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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }


	SbgErrorCode		errorCode = SBG_NO_ERROR;
	int					exitCode;

	//////////////params///////////////////
	this->IP_address = "192.168.50.50";
	this->input_port = 5678;
	this->output_port = 1234;


	if (this->IP_address.size() == 0){
		//
		// Create a serial interface to communicate with the PULSE
		//
		errorCode = sbgInterfaceSerialCreate(&this->sbgInterface, this->rs232.c_str(), this->baudrate);
	}
	else{
		//
		// Create a serial interface to communicate with the PULSE
		//
		errorCode = sbgInterfaceUdpCreate(&this->sbgInterface, sbgNetworkIpFromString(this->IP_address.c_str()), this->input_port, this->output_port);
	}

	if (errorCode == SBG_NO_ERROR)
		{
			comHandle = init_sbg_ekinox(&sbgInterface, &this->data_imu);
		}
		else
		{
			SBG_LOG_ERROR(errorCode, "unable to open serial interface");
			exitCode = EXIT_FAILURE;
		}

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;

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
	//
	// Try to read a frame
	//
	SbgErrorCode errorCode = sbgEComHandle(&comHandle);
	show_data(&data_imu);
	//
	// Test if we have to release some CPU (no frame received)
	//
	if (errorCode == SBG_NOT_READY)
	{
		//
		// Release CPU
		//
		sbgSleep(1);
	}
	else
	{
		SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");
	}
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
    // Utilizar s_handler en lugar de handler
	RoboCompIMU::DataImu *data = (RoboCompIMU::DataImu *)pUserArg;
    assert(pLogData);

	SBG_UNUSED_PARAMETER(pHandle);
	SBG_UNUSED_PARAMETER(pUserArg);

	if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0) {
		// Handle separately each received data according to the log ID
		switch (msg) {
			case SBG_ECOM_LOG_EKF_EULER:
				std::cout << "Euler status:"<< pLogData->ekfEulerData.status<<"*********Time: "<< pLogData->ekfEulerData.timeStamp << std::endl<<std::flush;
				data->rot.Roll = pLogData->ekfEulerData.euler[0];
				data->rot.Pitch = pLogData->ekfEulerData.euler[1];
				data->rot.Yaw = pLogData->ekfEulerData.euler[2];
				break;
			case SBG_ECOM_LOG_FAST_IMU_DATA:
				std::cout << "IMU status:"<< pLogData->fastImuData.status<<"*********Time: "<< pLogData->fastImuData.timeStamp << std::endl<<std::flush;
				data->acc.XAcc = pLogData->fastImuData.accelerometers[0];
				data->acc.YAcc = pLogData->fastImuData.accelerometers[1];
				data->acc.ZAcc = pLogData->fastImuData.accelerometers[2];
				data->gyro.XGyr = pLogData->fastImuData.gyroscopes[0];
				data->gyro.YGyr = pLogData->fastImuData.gyroscopes[1];
				data->gyro.ZGyr = pLogData->fastImuData.gyroscopes[2];


			default:
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

	//
	// Get device inforamtions
	//
	errorCode = sbgEComCmdGetInfo(pECom, &deviceInfo);

	//
	// Display device information if no error
	//
	if (errorCode == SBG_NO_ERROR)
	{
		char	calibVersionStr[32];
		char	hwRevisionStr[32];
		char	fmwVersionStr[32];		

		sbgVersionToStringEncoded(deviceInfo.calibationRev, calibVersionStr, sizeof(calibVersionStr));
		sbgVersionToStringEncoded(deviceInfo.hardwareRev, hwRevisionStr, sizeof(hwRevisionStr));
		sbgVersionToStringEncoded(deviceInfo.firmwareRev, fmwVersionStr, sizeof(fmwVersionStr));

		printf("      Serial Number: %0.9"PRIu32"\n",	deviceInfo.serialNumber);
		printf("       Product Code: %s\n",				deviceInfo.productCode);
		printf("  Hardware Revision: %s\n",				hwRevisionStr);
		printf("   Firmware Version: %s\n",				fmwVersionStr);
		printf("     Calib. Version: %s\n",				calibVersionStr);
		printf("\n");
	}
	else
	{
		SBG_LOG_WARNING(errorCode, "Unable to retrieve device information");
	}

	return errorCode;
}

/*!
 * Execute the ellipseMinimal example given an opened and valid interface.
 * 
 * \param[in]	pInterface							Interface used to communicate with the device.
 * \return											SBG_NO_ERROR if successful.
 */
SbgEComHandle  SpecificWorker::init_sbg_ekinox(SbgInterface *pInterface, RoboCompIMU::DataImu *data_imu )
{

	printf("Initializing sbg_ekinox\n");
	SbgErrorCode			errorCode = SBG_NO_ERROR;
	SbgEComHandle			comHandle;
		
	assert(pInterface);

	// Create the sbgECom library and associate it with the created interfaces
	errorCode = sbgEComInit(&comHandle, pInterface);

	// Test that the sbgECom has been initialized
	if (errorCode == SBG_NO_ERROR)
	{
		// Welcome message
		printf("Welcome to the ELLIPSE minimal example.\n");
		printf("sbgECom version %s\n\n", SBG_E_COM_VERSION_STR);

		// Query and display produce info, don't stop if there is an error
		print_product_info(&comHandle);

		// Showcase how to configure some output logs to 25 Hz, don't stop if there is an error
		errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);

		if (errorCode != SBG_NO_ERROR)
		{
			SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_IMU_DATA log");
		}

		errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8);

		if (errorCode != SBG_NO_ERROR)
		{
			SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_EKF_EULER log");
		}

		// Define callbacks for received data and display header
		sbgEComSetReceiveLogCallback(&comHandle, callback, data_imu);
	}
	else
	{
		SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
	}
	return comHandle;
}

void SpecificWorker::show_data(RoboCompIMU::DataImu *_data_imu)
{

	printf("Aceleration : %f, %f, %f\n",	_data_imu->acc.XAcc, _data_imu->acc.YAcc, _data_imu->acc.ZAcc);
	printf("Orientation : %f, %f, %f\n",	_data_imu->rot.Roll, _data_imu->rot.Pitch, _data_imu->rot.Yaw);

	printf("\n");
}


RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
//implementCODE

}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
//implementCODE

}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
//implementCODE

}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
//implementCODE

}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
//implementCODE

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

