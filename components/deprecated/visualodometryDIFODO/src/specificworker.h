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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "difodometrycamera.h"
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFile.h>

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 	

private:
	CDifodoCamera odo;
	const char *default_cfg_txt =
			"; ---------------------------------------------------------------\n"
			"; FILE: Difodo Parameters.txt\n"
			";\n"
			";  MJT @ JANUARY-2014\n"
			"; ---------------------------------------------------------------\n\n"

			"[DIFODO_CONFIG]\n\n"

			";cam_mode: 1 - 640x480, 2 - 320x240, 4 - 160x120 \n"
			"cam_mode = 2 \n\n"

			"Set the frame rate (fps) to 30 or 60 Hz \n"
			"fps = 30 \n\n"

			";Indicate the number of rows and columns. \n"
			"rows = 240 \n"
			"cols = 320 \n"
			"ctf_levels = 5 \n\n";
			
	InnerModel *innerModel;		
	
	#ifdef USE_QTGUI
		OsgView *osgView;
		InnerModelViewer *innerViewer;
		InnerModel *innerVisual;
	#endif
};

#endif

