/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
    listener->disconnect();
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
        /* Initialise the PTSDKListener object */
        bool isLogging;
        isLogging = false;

        listener = std::make_shared<PTSDKListener>(isLogging);

        sen0 = std::make_shared<PTSDKSensor>();
        sen1 = std::make_shared<PTSDKSensor>();
        sen2 = std::make_shared<PTSDKSensor>();
        sen3 = std::make_shared<PTSDKSensor>();

        /* Add all four sensors to the listener */
        listener->addSensor(sen0.get());
        listener->addSensor(sen1.get());
        listener->addSensor(sen2.get());
        listener->addSensor(sen3.get());

        int rate = 11520;//9600; 			// The rate of the serial connection
        int parity = 0; 			    	// 0=PARITY_NONE, 1=PARITY_ODD, 2=PARITY_EVEN
        char byteSize = 8; 				    // The number of bits in a byte
        int logFileRate = LOG_RATE_100;// The rate (Hz) to write to the log file: LOG_RATE_1000 or LOG_RATE_500 or LOG_RATE_100

        /* Connect to serial port */
        //int err = listener->connectAndStartListening(port.c_str(), rate, parity, byteSize, logFileRate);
        int err = listener->connect(port.c_str(), rate, parity, byteSize);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


        /* Check if the connection was successful */
        if (err)
        {
            printf("Could not connect to %s\n", port.c_str());
            return;
        }
        else
        {
            printf("Connected successfully to %s\n", port.c_str());
        }

        timer.start(100);
	}
}

void SpecificWorker::compute()
{
    /* Perform bias */
    if(listener->sendBiasRequest())
        printf("Successfully sent bias request.\n");
    else
    {
        printf("FAILED to send bias request.\n");
        return;
    }
    listener->readNextSample();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompContactile::FingerTips SpecificWorker::Contactile_getValues()
{
//implementCODE
    return RoboCompContactile::FingerTips();
}



/**************************************/
// From the RoboCompContactile you can use this types:
// RoboCompContactile::FingerTip
// RoboCompContactile::FingerTips

