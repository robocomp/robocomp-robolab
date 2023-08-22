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


CallbackHandler SpecificWorker::s_handler;

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
			errorCode = ellipseMinimalProcess(&sbgInterface);

			if (errorCode == SBG_NO_ERROR)
			{
				exitCode = EXIT_SUCCESS;
			}
			else
			{
				exitCode = EXIT_FAILURE;
			}

			sbgInterfaceDestroy(&sbgInterface);
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
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}

	
	
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

SbgErrorCode SpecificWorker::onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
    // Utilizar s_handler en lugar de handler
    return s_handler.onLogReceived(pHandle, msgClass, msg, pLogData, pUserArg);
}


/*!
 * Get and print product info.
 *
 * \param[in]	pECom					SbgECom instance.
 * \return								SBG_NO_ERROR if successful.
 */
SbgErrorCode  SpecificWorker::getAndPrintProductInfo(SbgEComHandle *pECom)
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
SbgErrorCode  SpecificWorker::ellipseMinimalProcess(SbgInterface *pInterface)
{
	SbgErrorCode			errorCode = SBG_NO_ERROR;
	SbgEComHandle			comHandle;
		
	assert(pInterface);

	//
	// Create the sbgECom library and associate it with the created interfaces
	//
	errorCode = sbgEComInit(&comHandle, pInterface);

	//
	// Test that the sbgECom has been initialized
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Welcome message
		//
		printf("Welcome to the ELLIPSE minimal example.\n");
		printf("sbgECom version %s\n\n", SBG_E_COM_VERSION_STR);

		//
		// Query and display produce info, don't stop if there is an error
		//
		getAndPrintProductInfo(&comHandle);

		//
		// Showcase how to configure some output logs to 25 Hz, don't stop if there is an error
		//
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


		//
		// Define callbacks for received data and display header
		//
		//sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

		sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);
		

		printf("Euler Angles display with estimated standard deviation - degrees\n");

		//
		// Loop until the user exist
		//
		while (1)
		{
			//
			// Try to read a frame
			//
			errorCode = sbgEComHandle(&comHandle);
			std::cout << "angle: " << s_handler.getAngle() << std::endl;
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

		//
		// Close the sbgEcom library
		//
		sbgEComClose(&comHandle);
	}
	else
	{
		SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
	}

	return errorCode;
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

