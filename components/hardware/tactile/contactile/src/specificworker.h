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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "stdafx.h"
#include <stdio.h>

#ifndef PTSDKCONSTANTS_H
#include "PTSDKConstants.h"
#endif
#ifndef PTSDKLISTENER_H
#include "PTSDKListener.h"
#endif
#ifndef PTSDKSENSOR_H
#include "PTSDKSensor.h"
#endif

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        RoboCompContactile::FingerTips Contactile_getValues();


    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        bool startup_check_flag;

        //
        std::shared_ptr<PTSDKListener> listener;
        std::shared_ptr<PTSDKSensor> sen0, sen1, sen2, sen3;

        // port
        std::string port = "/dev/ttyACM0";
};

#endif
