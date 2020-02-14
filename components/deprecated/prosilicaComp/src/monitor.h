/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef MONITOR_H
#define MONITOR_H

#include <Ice/Ice.h>
#include <QtCore>
#include "worker.h"
#include "config.h"
#include <qlog/qlog.h>
#include <CommonBehavior.h>

/**
       \brief
       @author authorname
*/
class Monitor : public QThread
{
  Q_OBJECT
  
  public:
	Monitor(Worker *_worker, Ice::CommunicatorPtr _communicator);
	~Monitor();
	
	void readConfig(RoboCompCommonBehavior::ParameterList &params );
	void run();
	void initialize();
    //CommonBehavior
	int getPeriod();
	void setPeriod(int period);
	void killYourSelf();
	int timeAwake();
	RoboCompCommonBehavior::ParameterList getParameterList();
	void setParameterList(RoboCompCommonBehavior::ParameterList l);
	RoboCompCommonBehavior::State getState();
  private:
	int period;
	Worker *worker;
	Ice::CommunicatorPtr communicator;
	QTime initialTime;
	RoboCompCommonBehavior::ParameterList config_params;
	RoboCompCommonBehavior::State state;
	
	bool sendParamsToWorker(RoboCompCommonBehavior::ParameterList params);
	bool checkParams(RoboCompCommonBehavior::ParameterList l);
	bool configGetString( const std::string name, std::string &value,  const std::string default_value, QStringList *list = NULL);
  signals:
	void kill();
};

#endif // MONITOR_H
