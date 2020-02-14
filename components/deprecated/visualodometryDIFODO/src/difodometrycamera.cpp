/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "difodometrycamera.h"
#include <mrpt/utils/types_math.h> // Eigen (with MRPT "plugin" in BaseMatrix<>)
#include <mrpt/system/filesystem.h>


using namespace Eigen;
using namespace std;
using namespace mrpt;


void CDifodoCamera::loadConfiguration(const utils::CConfigFileBase &ini , RoboCompRGBD::RGBDPrx _rgbd_proxy)
{
	rgbd_proxy = _rgbd_proxy;
	
	fovh = M_PI*58.6/180.0;
	fovv = M_PI*45.6/180.0;
	downsample = 1;
	cam_mode = ini.read_int("DIFODO_CONFIG", "cam_mode", 2, true);
	rows = ini.read_int("DIFODO_CONFIG", "rows", 240, true);
	cols = ini.read_int("DIFODO_CONFIG", "cols", 320, true);
	fps = ini.read_int("DIFODO_CONFIG", "fps", 30, false);
	ctf_levels = ini.read_int("DIFODO_CONFIG", "ctf_levels", 5, true);


	//			Resize Matrices and adjust parameters
	//=========================================================
	width = 640/(cam_mode*downsample);
	height = 480/(cam_mode*downsample);
	repr_level = utils::round(log(float(width/cols))/log(2.f));

	//Resize pyramid
    const unsigned int pyr_levels = utils::round(log(float(width/cols))/log(2.f)) + ctf_levels;
    depth.resize(pyr_levels);
    depth_old.resize(pyr_levels);
    depth_inter.resize(pyr_levels);
	depth_warped.resize(pyr_levels);
    xx.resize(pyr_levels);
    xx_inter.resize(pyr_levels);
    xx_old.resize(pyr_levels);
	xx_warped.resize(pyr_levels);
    yy.resize(pyr_levels);
    yy_inter.resize(pyr_levels);
    yy_old.resize(pyr_levels);
	yy_warped.resize(pyr_levels);
	transformations.resize(pyr_levels);

	for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = width/s; rows_i = height/s;
        depth[i].resize(rows_i, cols_i);
        depth_inter[i].resize(rows_i, cols_i);
        depth_old[i].resize(rows_i, cols_i);
        depth[i].assign(0.0f);
        depth_old[i].assign(0.0f);
        xx[i].resize(rows_i, cols_i);
        xx_inter[i].resize(rows_i, cols_i);
        xx_old[i].resize(rows_i, cols_i);
        xx[i].assign(0.0f);
        xx_old[i].assign(0.0f);
        yy[i].resize(rows_i, cols_i);
        yy_inter[i].resize(rows_i, cols_i);
        yy_old[i].resize(rows_i, cols_i);
        yy[i].assign(0.0f);
        yy_old[i].assign(0.0f);
		transformations[i].resize(4,4);

		if (cols_i <= cols)
		{
			depth_warped[i].resize(rows_i,cols_i);
			xx_warped[i].resize(rows_i,cols_i);
			yy_warped[i].resize(rows_i,cols_i);
		}
    }

	depth_wf.setSize(height,width);
}

bool CDifodoCamera::openCamera()
{
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	printf("Initialization:\n %s\n", openni::OpenNI::getExtendedError());
	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//				Create Depth channel
	//-----------------------------------------------------
	rc = depth_ch.create(device, openni::SENSOR_DEPTH);

	//Display video modes
	openni::VideoMode vm;
	printf("Camera depth modes are listed:\n");
	for(int i=0; i<depth_ch.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	{
		vm = depth_ch.getSensorInfo().getSupportedVideoModes()[i];
		cout << "depth mode :" << vm.getResolutionX() << "x" << vm.getResolutionY() << "@" << vm.getFps() << "Hz using format " << vm.getPixelFormat() << endl;
	}

	//					Set video mode
	//------------------------------------------------------
	const unsigned int resh = 640/cam_mode;
	const unsigned int resv = 480/cam_mode;
	video_options = depth_ch.getVideoMode();
	video_options.setResolution(resh,resv);
	video_options.setFps(fps);
	rc = depth_ch.setVideoMode(video_options);
	rc = depth_ch.setMirroringEnabled(false);

	video_options = depth_ch.getVideoMode();
	printf("\nResolution (%d, %d) \n", video_options.getResolutionX(), video_options.getResolutionY());

	if (rc == openni::STATUS_OK)
	{
		rc = depth_ch.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth_ch.destroy();
		}
	}
	else
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());

	return 0;
}
void CDifodoCamera::loadFrame()
{
	RoboCompRGBD::DepthSeq depth;
	static RoboCompDifferentialRobot::TBaseState bState;
	static RoboCompJointMotor::MotorStateMap hState;

	const int height = 240;
	const int width = 320;
	
	try
	{	
		rgbd_proxy->getDepth(depth, hState, bState);
		
		float *pDepth = &depth[0];
		for (int yc = height-1; yc >= 0; --yc)
		{
			for (int xc = width-1; xc >= 0; --xc)
			{
				if (*pDepth < 4500.f)
					depth_wf(yc,xc) = 0.001f*(*pDepth);
				else
					depth_wf(yc,xc) = 0.f;
				++pDepth;
				//cout << *pDepth <<  " " << endl;
			}
		}
	}
	catch(const Ice::Exception &ex)
	{ cout << ex << endl;};
	
}

// void CDifodoCamera::loadFrame()
// {
// 	cout << __FUNCTION__ << endl;
// 	//Read the newest frame
// 	openni::VideoFrameRef framed;
// 	depth_ch.readFrame(&framed);
// 
// 	const int height = framed.getHeight();
// 	const int width = framed.getWidth();
// 
// 	//Store the depth values
// 	const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
// 	int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);
// 
// 	for (int yc = height-1; yc >= 0; --yc)
// 	{
// 		const openni::DepthPixel* pDepth = pDepthRow;
// 		for (int xc = width-1; xc >= 0; --xc, ++pDepth)
// 		{
// 			if (*pDepth < 4500.f)
// 				depth_wf(yc,xc) = 0.001f*(*pDepth);
// 
// 			else
// 				depth_wf(yc,xc) = 0.f;
// 		}
// 
// 		pDepthRow += rowSize;
// 	}
// }



void CDifodoCamera::CreateResultsFile()
{
	try
	{
		// Open file; find the first free file-name.
		char	aux[100];
		int     nFile = 0;
		bool    free_name = false;

		system::createDirectory("./difodo.results");

		while (!free_name)
		{
			nFile++;
			sprintf(aux, "./difodo.results/experiment_%03u.txt", nFile );
			free_name = !system::fileExists(aux);
		}

		// Open log file:
		f_res.open(aux);

		clock.Tic();

		printf(" Saving results to file: %s \n", aux);
	}
	catch (...)
	{
		printf("Exception found trying to create the 'results file' !!\n");
	}
}



void CDifodoCamera::closeCamera()
{
	depth_ch.destroy();
	openni::OpenNI::shutdown();
}

void CDifodoCamera::reset()
{
	//Reset Difodo
	loadFrame();
	buildImagePyramid();
	loadFrame();
	odometryCalculation();

	cam_pose.setFromValues(0,0,1.5,0,0,0);
	cam_oldpose = cam_pose;

	//Reset scene
	//scene = window.get3DSceneAndLock();
	//CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
	//traj_lines_odo->clear();
	//CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(0);
	//traj_points_odo->clear();
	//window.unlockAccess3DScene();

	//updateScene();
}
// void CDifodoCamera::reset()
// {
// 	//Reset Difodo
// 	loadFrame();
// 	buildImagePyramid();
// 	loadFrame();
// 	odometryCalculation();
// 
// 	cam_pose.setFromValues(0,0,1.5,0,0,0);
// 	cam_oldpose = cam_pose;
// 
// 	//Reset scene
// 	scene = window.get3DSceneAndLock();
// 	CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
// 	traj_lines_odo->clear();
// 	CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(0);
// 	traj_points_odo->clear();
// 	window.unlockAccess3DScene();
// 
// 	updateScene();
// }

void CDifodoCamera::writeTrajectoryFile()
{
	mrpt::math::CQuaternionDouble quat;
	mrpt::poses::CPose3D auxpose, transf;
	transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

	auxpose = cam_pose - transf;
	auxpose.getAsQuaternion(quat);
	
	f_res << clock.Tac() << " ";
	f_res << cam_pose[0] << " ";
	f_res << cam_pose[1] << " ";
	f_res << cam_pose[2] << " ";
	f_res << quat(2) << " ";
	f_res << quat(3) << " ";
	f_res << -quat(1) << " ";
	f_res << -quat(0) << endl;
}

