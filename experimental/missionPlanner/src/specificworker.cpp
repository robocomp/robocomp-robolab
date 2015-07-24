/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus/etc/autonomyLab.xml");
	target.x=29239249;
	target.z=29239249;
	updateState(State::GOTO, mutex_state);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	  state = State::STOP;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	asrpublish_proxy->newText("I go to the party");
	Tag tagObjetive;
	updateInnerModel(innerModel);

	updateState(State::GOTO, mutex_state);
	switch (state)
	{
		case State::GOTO:
		//	if(tagList.getTagR(1,tagObjetive) && tagObjetive.isValid(3000))
			{
		//		qDebug()<<tagObjetive.x()<<tagObjetive.y()<<tagObjetive.z()<<endl;
		//		tagObjetive.coords = innerModel->transform("world", QVec::vec3(tagObjetive.x(), tagObjetive.y(), tagObjetive.z()), "rgbd_transform");

// 				   list=sam.sampleFreeSpaceR2Uniform(box);
		//		target.x = tagObjetive.coords.x();
				target.y = 0;
		//		target.z = tagObjetive.coords.z();
				target.x = 1000;
				target.z = 4000;
			try
			{
				qDebug() << "MOVING :-)";
				asrpublish_proxy->newText("I go to the party");
				//trajectoryrobot2d_proxy->stop();
				trajectoryrobot2d_proxy->go(target);
				updateState(State::GOTO, mutex_state);
				printf("Go to ( %f , %f , %f )",target.x,target.y,target.z);
			}
			catch(const Ice::Exception &ex){
				cout << ex << endl;
			}

			}
		//	else
		//	{
		//		RoboCompTrajectoryRobot2D::NavState s = trajectoryrobot2d_proxy->getState();
		//		if(s.state != "IDLE") updateState(State::GOTO, mutex_state);	
		//		else updateState(State::LOST, mutex_state);
		//	}
			break;
		case State::LOST:
			qDebug()<<"lost :-(";
			stop();
			updateState(State::SEARCHING, mutex_state);
			clock.restart();
			break;

		case State::SEARCHING:
			if(tagList.getTag(1,tagObjetive) && tagObjetive.isValid(3000))
			{
				updateState(State::GOTO, mutex_state);
				
				stop();
			}
			//if(clock.elapsed() > 2000 && clock.isValid()) search();
			break;

		case State::STOP:
			qDebug() << "Idle" << endl;
			break;

		default:
			qDebug() << "Connection Failed!" << endl;
			break;
	}

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Updates an InnerModel from values read from the robot. Reads laserData too.
 * 
 * @param inner InnerModel that is to be updated
 * @return bool
 */
bool SpecificWorker::updateInnerModel(InnerModel *innerModel)
{
	// TODO sacar del trajectory2d
	return true;
}
////////////////


void SpecificWorker::stop()
{
	trajectoryrobot2d_proxy->stop();
}

void SpecificWorker::updateState(State st, QMutex& mutex_state)
{
	QMutexLocker ml(&mutex_state);
	state = st;
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	
	for(auto tag : tags)
	{	
		qDebug()<<tag.id;
		Tag t(tag.id, tag.tx, tag.ty, tag.tz);
		tagList.addTag(t);
	}

}
