/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

	sensor.setDevice(configLoader.get<std::string>("device"));
	if (!sensor.open()) {
		std::cerr << "Failed to open sensor: " << strerror(errno) << std::endl;
		exit(1);
	}
}


void SpecificWorker::compute()
{
    BotaFTSensor::FTData data;

    switch (sensor.read(data))
    {
        case BotaFTSensor::VALID:
            updateFTData(data, false);
            consecutiveErrors = 0;
            break;

        case BotaFTSensor::STATUS_ERROR:
            if (data.status.overrange)
                qWarning() << "[BotaFTSensor] Overrange detected";
            if (data.status.app_took_too_long)
                qWarning() << "[BotaFTSensor] App took too long";
            if (data.status.invalid_measurements)
                qWarning() << "[BotaFTSensor] Invalid measurements";
            consecutiveErrors++;
            break;

        case BotaFTSensor::CRC_ERROR:
            qWarning() << "[BotaFTSensor] CRC error #" << sensor.getCrcErrorCount();
            consecutiveErrors++;
            break;

        case BotaFTSensor::SYNC_ERROR:
            qWarning() << "[BotaFTSensor] Synchronization lost, attempting to resync...";
            consecutiveErrors++;
            break;

        case BotaFTSensor::NO_DATA:
            break;
    }

    if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS)
    {
        qWarning() << "[BotaFTSensor] Too many consecutive errors, attempting to reset the sensor...";
        sensor.close();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (sensor.open())
        {
            sensor.tare(100);
            consecutiveErrors = 0;
            qInfo() << "[BotaFTSensor] Sensor reset successfully";
        }
        else
        {
            qCritical() << "[BotaFTSensor] Failed to reconnect to the sensor: " << strerror(errno);
            consecutiveErrors = 0;
        }
    }
}

void SpecificWorker::updateFTData(const BotaFTSensor::FTData& data, bool info)
{
	lastSensorData.fx = data.forces[0];
	lastSensorData.fy = data.forces[1];
	lastSensorData.fz = data.forces[2];
	lastSensorData.tx = data.forces[3];
	lastSensorData.ty = data.forces[4];
	lastSensorData.tz = data.forces[5];
	lastSensorData.temperature = data.temperature;
	lastSensorData.timestamp = data.timestamp;
	lastSensorData.valid = true;

	if (info)
	{
		qInfo() << "[BotaFTSensor] New data: Fx=" << lastSensorData.fx
				<< " Fy=" << lastSensorData.fy
				<< " Fz=" << lastSensorData.fz
				<< " Tx=" << lastSensorData.tx
				<< " Ty=" << lastSensorData.ty
				<< " Tz=" << lastSensorData.tz
				<< " Temp=" << lastSensorData.temperature
				<< " Timestamp=" << lastSensorData.timestamp;
	}
}


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

RoboCompForceTorqueSensor::SensorData SpecificWorker::ForceTorqueSensor_getSensorData()
{
	return lastSensorData;
}



/**************************************/
// From the RoboCompForceTorqueSensor you can use this types:
// RoboCompForceTorqueSensor::SensorData

