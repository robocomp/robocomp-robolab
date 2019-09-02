/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/vision/CDifodo.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/round.h>
#include <iostream>
#include <RGBD.h>

#if defined(MRPT_OS_LINUX) && !defined(linux)
#   define linux 1   // Seems to be required by OpenNI.h
#endif
#include <OpenNI.h>


class CDifodoCamera : public mrpt::vision::CDifodo {
public:

	std::ofstream		f_res;

	bool save_results;

	/** Constructor. */
	CDifodoCamera() : mrpt::vision::CDifodo()
	{
		save_results = 0;
	}

	/** Initialize the visual odometry method */
	void loadConfiguration( const mrpt::utils::CConfigFileBase &ini , RoboCompRGBD::RGBDPrx _rgbd_proxy);

	/** Open camera */
	bool openCamera();

	/** Close camera */
	void closeCamera();

	/** Capture a new depth frame */
	void loadFrame();

	/** Create a file to save the estimated trajectory */
	void CreateResultsFile();

	/** A pre-step that should be performed before starting to estimate the camera velocity.
	  * It can also be called to reset the estimated trajectory and pose */
	void reset();

	/** Save the pose estimation following the format of the TUM datasets:
	  * 
	  * 'timestamp tx ty tz qx qy qz qw'
	  *
	  * Please visit http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats for further details.*/  
	void writeTrajectoryFile();

private:
	
	RoboCompRGBD::RGBDPrx rgbd_proxy;
	unsigned int repr_level;

	// OpenNI variables to manage the camera
	openni::Status		rc;
	openni::Device		device;
	openni::VideoMode	video_options;
	openni::VideoStream depth_ch;

	/** Clock used to save the timestamp */
	mrpt::utils::CTicTac clock;
};
