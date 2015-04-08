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
#ifndef CONST_H
#define CONST_H

#define PROGRAM_NAME        "JoyStickHandler"
#define PROGRAM_FILE_NAME   "joystickhandler"

//OJO tiene que ser simétrico de momento
#define JOYSTICK_MAX_RAW 32767.
#define JOYSTICK_MIN_RAW -32767.
#define JOYtoROBOT_ROT	( 1 / JOYSTICK_MAX_RAW)
#define JOYtoROBOT_ADV	( 1 / JOYSTICK_MAX_RAW)
#define JOYSTICK_CENTER  0.1
#define DEFAULT_BASE_SERVER_PROXY_STRING       "OmniRobotProxy"

#define JOYSTICK_EVENT_TYPE_NULL     0x00
#define JOYSTICK_EVENT_TYPE_BUTTON   JS_EVENT_BUTTON
#define JOYSTICK_EVENT_TYPE_AXIS     JS_EVENT_AXIS


#endif
