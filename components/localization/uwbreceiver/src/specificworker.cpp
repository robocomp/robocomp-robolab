/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit computetofinalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
    params = _params;

    defaultMachine.start();
    
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	xPos = 0;
	zPos = 0;
	timer.start(Period);
	emit this->initializetocompute();

}

void SpecificWorker::compute_initial_pose(int ntimes)
{
	QPointF posL, posR;
	float xMed = 0;
	float zMed = 0;
	float angle = 0;
	for (int cont=0; cont < ntimes; cont++)
	{
        posL = readData(left_device);
		if (ndevices == 2)
		{
			posR = readData(right_device);
			xMed += (posL.x() + posR.x()) / 2.;
			zMed += (posL.y() + posR.y()) / 2.;
			angle += QLineF(posR, posL).angle();
		}
		else
		{
			xMed += posL.x();
			zMed += posL.y();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	xPos = xMed / ntimes;
	zPos = zMed / ntimes;
	ryPos = angle / ntimes;
	std::cout << "initial pose: "<< xPos << " " << zPos << " " << ryPos << std::endl;
}
void SpecificWorker::compute()
{
	QPointF posL = readData(left_device);

	try
	{
		RoboCompFullPoseEstimation::FullPose pose;
		pose.source = "uwb";
		pose.x = posL.x();
		pose.z = posL.y();
		if (ndevices == 2)
		{
			QPointF posR = readData(right_device);
			pose.x = (posL.x() + posR.x()) / 2.;
			pose.z  = (posL.y() + posR.y()) / 2.;
			pose.ry = degreesToRadians(QLineF(posR, posL).angle()-180);
			xPos = pose.x;
			zPos = pose.z;
			ryPos = pose.ry;
		}
		std::cout << "Pose read(x,z,ry): "<< pose.x << " " << pose.z << " " << pose.ry << std::endl;
		fullposeestimationpub_pubproxy->newFullPose(pose);
	}
	catch(const Ice::Exception &e)
	{
		std::cout <<"Error pose publication, check Ice connection: "<< e << std::endl;
	}

}


void SpecificWorker::sm_compute()
{
	//std::cout<<"Entered state compute"<<std::endl;
    
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
	//Check devices number => If there is just one, left variable is used
	ndevices = std::stoi( params["ndevices"].value);
	left_device.setPortName(QString::fromStdString(params["left"].value));
	if(!left_device.open(QIODevice::ReadWrite))
	{
		std::cout << "Error opening left_device: " << params["left"].value << std::endl;
 		exit(-1); 
	}
	left_device.setBaudRate(QSerialPort::Baud115200);
	if (ndevices == 2)
	{
		right_device.setPortName(QString::fromStdString(params["right"].value));
		if(!right_device.open(QIODevice::ReadWrite))
		{
			std::cout << "Error opening left_device: " << params["right"].value << std::endl;
			exit(-1); 
		}
		right_device.setBaudRate(QSerialPort::Baud115200);
	}
	compute_initial_pose(std::stoi( params["initial_reading"].value));
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}



QPointF SpecificWorker::readData(QSerialPort &serial)
{
	QByteArray responseData;
	responseData.append((char)2);
    responseData.append((char)0);

//	for(int i=0; i<1; i++)
//	{
		serial.write(responseData);
  		if (serial.waitForBytesWritten(200)) {}
          //qDebug() << "escrito ok";
		int numRead = 0, numReadTotal = 0;

		char *b;b = new char[18];
		for (;;) 
		{
			if (serial.waitForReadyRead(100)) 
			{
				numRead  = serial.read(b + numReadTotal, serial.bytesAvailable());
				numReadTotal += numRead;
			}
			if(numReadTotal >= 18)
			break;
		}

		// for(int i=0; i<18; i++)
		// 	std::cout << (int)b[i] << " "; 
		// std::cout <<  std::endl;

		if(!(b[0] == 64 and b[1] == 1 and b[2] == 0 and b[3] == 65))
		{	
			qDebug() << "Bad sequence, aborting";
			return QPointF();
		}

		int x = bitsToInt((unsigned char*)b,5,true);
		int y = bitsToInt((unsigned char*)b,9,true);
		int z = bitsToInt((unsigned char*)b,13,true);
		//qDebug() << x << y << z;
		//qDebug() << "---------------------";
		delete b;
		serial.flush();
		//usleep(100000);
		return(QPointF(x,y));
//	}
}

//UTILITIES
float SpecificWorker::degreesToRadians(const float angle_)
{	
	float angle = angle_ * 2*M_PI / 360;
	if(angle > M_PI)
   		return angle - M_PI*2;
	else if(angle < -M_PI)
   		return angle + M_PI*2;
	else return angle;
}

int SpecificWorker::bitsToInt( const unsigned char* bits, uint init, bool little_endian)
	{
		int result = 0;
		if (little_endian)
			for (int n = sizeof( result ); n >= 0; n--)
			result = (result << 8) + bits[ init + n ];
		else
			for (unsigned n = 0; n < sizeof( result ); n++)
			result = (result << 8) + bits[ n ];
		return result;
	}


//***********INTERFACE*****************//

FullPose SpecificWorker::FullPoseEstimation_getFullPose()
{
	RoboCompFullPoseEstimation::FullPose pose;
	pose.x = xPos;
	pose.z = zPos;
	pose.ry = ryPos;
	return pose;
}
void SpecificWorker::FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz)
{
	qDebug()<<"Not implemented yet";
}

