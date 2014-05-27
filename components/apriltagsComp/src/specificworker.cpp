/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx), m_tagCodes(::AprilTags::tagCodes16h5)
{
	m_tagDetector = NULL;
	m_draw = true;

/*	try{		camParams = camera_proxy->getCamParams();	}
	catch(const Ice::Exception &e)	{	std::cout << e << std::endl;}*/
 /*  RoboCompRGBD::TRGBDParams rgbdParams;
   try{        rgbdParams = rgbd_proxy->getRGBDParams();   }
   catch(const Ice::Exception &e)  {   std::cout << e << std::endl;}
   qDebug() << rgbdParams.color.focal << rgbdParams.color.width << rgbdParams.color.height;*/
//	qDebug() << "Read cam params" << camParams.width << camParams.height << camParams.focal;
	//m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
	///FIX ALL THIS MESS!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
/*	m_width = camParams.width;
	m_height = camParams.height;
	m_tagSize = 0.166;
	m_fx =camParams.focal;
	m_fy =camParams.focal;
	m_px = m_width/2;
	m_py = m_height/2;*/

	m_width = 640;
	m_height = 480;
  
	//m_tagSize = 0.374;
	//m_tagSize = 0.397;
	
	/*m_fx =480;
	m_fy =480;
	*/
// 	m_fx =583;
//     m_fy =583;
	m_fx = 400;
	m_fy = 400;
	m_px = m_width/2;
	m_py = m_height/2;

	//cv::namedWindow("AprilTags", 1);
	image_gray.create(480,640,CV_8UC1);
	image_color.create(480,640,CV_8UC3);
	printf("fin constr\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		INPUTIFACE = Camera;
		RoboCompCommonBehavior::Parameter par = params.at("InputInterface") ;
		if (par.value == "RGBD")
		{
			INPUTIFACE = RGBD;
		}
		else if ( par.value == "RGBDBus")
		{
			INPUTIFACE = RGBDBus;
		}
		else if ( par.value == "Camera")
		{
			INPUTIFACE = Camera;
		}
		
		else
			qFatal("InputInterface");
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("AprilTagsFamily") ;
		if (par.value == "tagCodes16h5")
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);
		else if ( par.value == "tagCodes25h7")
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h7);
		else if ( par.value == "tagCodes25h9")
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h9);
		else if ( par.value == "tagCodes36h11")
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
		else if ( par.value == "tagCodes36h9")
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h9);	
		else
			m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);				
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("AprilTagsSize");
		Q_ASSERT(par.value > 0 and par.value < 5);
		m_tagSize = QString::fromStdString(par.value).toFloat();
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("CameraName");
		camera_name=par.value;
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innermodel_path=par.value;
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	
	timer.start(100);
	
// 	innermodel = new InnerModel(innermodel_path);
//  	m_fx = 400;//innermodel->getCameraFocal(camera_name.c_str());
//  	m_fy = 400;//innermodel->getCameraFocal(camera_name.c_str());
// 	printf("FOCAL LENGHT: %f\n", float(innermodel->getCameraFocal(camera_name.c_str())) );

	return true;
}

void SpecificWorker::getImageGray()
{
	if( INPUTIFACE == Camera)
	{
		//For cameras
		RoboCompCamera::imgType img;
		try
		{
			camera_proxy->getYImage(0,img, cState, bState);		
			memcpy(image_gray.data , &img[0], 640*480*sizeof(uchar));
		}
		catch(const Ice::Exception &e)
		{
			std::cout << "Error reading from Camera" << e << std::endl;
		}
	}
	else if( INPUTIFACE == RGBD)
	{
		try
		{
			//For RGBD
			RoboCompRGBD::ColorSeq colorseq;
			RoboCompRGBD::DepthSeq depthseq;
			rgbd_proxy->getRGB(colorseq, hState, bState);
			memcpy(image_color.data , &colorseq[0], 640*480*3);
			cv::cvtColor(image_color, image_gray, CV_RGB2GRAY); 
		}
		catch(const Ice::Exception &e)
		{
			std::cout << "Error reading form RGBD" << e << std::endl;
		}
	}
	else if( INPUTIFACE == RGBDBus)
	{
		qFatal("Not implemented yet!");
	}
	else
		qFatal("Input device not defined. Please specify one in the config file");
}
void SpecificWorker::compute()
{
	static int frame = 0;
	double last_t = tic();

	getImageGray();

	vector< ::AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray); 

	// print out each detection
	cout << detections.size() << " tags detected:" << endl;

	print_detection(detections);

	if (m_draw) 
	{
		for (uint i=0; i<detections.size(); i++) 
		{
			detections[i].draw(image_gray);
		}
// 		imshow("AprilTags", image_gray); // OpenCV call
	}

	// print out the frame rate at which image frames are being processed
	frame++;
	if (frame % 10 == 0) 
	{
		double t = tic();
		cout << "  " << 1./(t-last_t) << " fps" << endl;
		last_t = t;
	}
}

void SpecificWorker::print_detection(vector< ::AprilTags::TagDetection> detections)
{
	detections2send.resize(detections.size());
	listaDeMarcas.resize(detections.size());
	
	for(uint i=0; i<detections.size(); i++)
	{
		::AprilTags::TagDetection detection = detections[i];
		
		cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ")";

		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);

		Eigen::Matrix3d F;
		F <<
		1, 0,  0,
		0,  -1,  0,
		0,  0,  1;
		Eigen::Matrix3d fixed_rot = F*rotation;
		double yaw, pitch, roll;
		wRo_to_euler(fixed_rot, yaw, pitch, roll);

		cout << "  distance=" << translation.norm()
			 << "m, x=" << translation(0)
			 << ", y=" << translation(1)
			 << ", z=" << translation(2)
			 << ", yaw=" << yaw
			 << ", pitch=" << pitch
			 << ", roll=" << roll
			 << endl;

		// Also note that for SLAM/multi-view application it is better to
		// use reprojection error of corner points, because the noise in
		// this relative pose is very non-Gaussian; see iSAM source code
		// for suitable factors.
		// fill in tag to send
		RoboCompAprilTags::tag t;
		RoboCompGetAprilTags::marca mar;

		t.id=detection.id;
		t.tx=-translation(1)*1000.;
		t.ty=translation(2)*1000.;
		t.tz=translation(0)*1000.;
		//Change the x,y,z rotation values to match robocomp's way
		t.rx=-roll;
		t.ry=-pitch;
		t.rz=-yaw;

		memcpy(&mar, &t, sizeof(RoboCompGetAprilTags::marca));
		mutex->lock();
		detections2send[i]=t;
		listaDeMarcas[i]=mar;
		mutex->unlock();
	}
		
	try
	{
		printf("<<");
		apriltags->newAprilTag(detections2send);
		printf(">>\n");
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

		
/**
* Normalize angle to be within the interval [-pi,pi].
*/
double SpecificWorker::standardRad(double t)
{
	if (t >= 0.) {
		t = fmod(t+M_PI, TWOPI) - PI;
	} else {
		t = fmod(t-M_PI, -TWOPI) + PI;
	}
	return t;
}

void SpecificWorker::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) 
{
	yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
	roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double SpecificWorker::tic() 
{
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/////////
/// IMPLEMENTS
////////

listaMarcas SpecificWorker::checkMarcas()
{
  QMutexLocker locker(mutex);
  return  listaDeMarcas;
}
