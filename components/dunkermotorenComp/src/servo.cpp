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
#include "servo.h"

Servo::Servo( RoboCompJointMotor::MotorParams params_ )
{
  params = params_;
  data.name = QString::fromStdString(params.name);
  data.busId = params.busId;
  data.targetVelocityRads = 0.f;
  data.currentVelocityRads = 0.f;
  data.antPosRads = 0;
  data.currentPos = 0;
  data.currentPosRads = 0;
  data.currentPower = 0;
  data.isMoving = false;
  data.mode = -1;
  pendAct = true;
}

Servo::~Servo()
{}

void Servo::setMotorRanges(float step_range, float degrees_range, float speed_step_range, float max_speed_rads)
{
	RAW_STEPS_RANGE=step_range;
	RAW_DEGREES_RANGE=degrees_range;
 	RAW_RADIANS_RANGE=RAW_DEGREES_RANGE * M_PI / 180.;
	RAW_SPEED_STEPS_RANGE=speed_step_range;
	MAX_SPEED_RADS=max_speed_rads;
}

void Servo::setPendiente(int pos)
{
	this->pendAct = true;
	if (fabs(pos - data.currentPos)>ERR_POSITION_STEPS)
		this->newCommand=true;
	else
		this->newCommand=false;
}

bool Servo::pendiente()
{ 
  return pendAct; 
}

float Servo::steps2Rads(int p)
{
	if (params.invertedSign)
		 return ((float)(p-params.zeroPos)) * (RAW_RADIANS_RANGE/RAW_STEPS_RANGE);
	else
		return ((float)(p-params.zeroPos)) * (-RAW_RADIANS_RANGE/RAW_STEPS_RANGE);
	
}

int Servo::rads2Steps(float r)
{
	if (params.invertedSign)
		return( (int)rint((RAW_STEPS_RANGE / RAW_RADIANS_RANGE) * r + params.zeroPos)) ;
	else 
		return( (int)rint(-(RAW_STEPS_RANGE / RAW_RADIANS_RANGE) * r + params.zeroPos)) ;
}

int Servo::rads2StepsVel(float r)
{
	if (params.invertedSign)
		return (int)rint((RAW_STEPS_RANGE / RAW_RADIANS_RANGE) * r);
	else 
		return (int)rint(-(RAW_STEPS_RANGE / RAW_RADIANS_RANGE) * r);
}

 int Servo::radsPerSec2Steps(float rs)
 {
	//Max speed
	printf("Servo::radsPerSec2Steps: NOT IMPLEMENTED");
	exit(-1);
	return 1024;
	// int res = (int)rint((RAW_SPEED_STEPS_RANGE / MAX_SPEED_RADS) * r );
	// if (res > 1024) return 1024  ;
	// if (res < 0 return 0;
 }
 
