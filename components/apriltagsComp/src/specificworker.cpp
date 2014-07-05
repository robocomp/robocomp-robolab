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
SpecificWorker::SpecificWorker(MapPrx& mprx) : 
	GenericWorker(mprx),
	m_tagDetector(NULL),
	m_tagCodes(::AprilTags::tagCodes16h5),
	m_draw(true)
{
	m_width = 640;
	m_height = 480;
	m_px = m_width/2;
	m_py = m_height/2;

	image_gray.create(480,640,CV_8UC1);
	image_color.create(480,640,CV_8UC3);
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
		RoboCompCommonBehavior::Parameter par = params.at("InputInterface");
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
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("CameraName");
		camera_name=par.value;
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innermodel_path=par.value;
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	innermodel = new InnerModel(innermodel_path);
	
 	m_fx = innermodel->getCameraFocal(camera_name.c_str());
  	m_fy = innermodel->getCameraFocal(camera_name.c_str());

	qDebug() << QString::fromStdString(innermodel_path) << " " << QString::fromStdString(camera_name);
	qDebug() << "FOCAL LENGHT:" << innermodel->getCameraFocal(camera_name.c_str());

	//Reading id sets size to create a map 
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:0-10");
		Q_ASSERT(par.value > 0 and par.value < 500);
		for(int i=0;i<=10; i++)
			tagsSizeMap.insert( i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) {  qFatal("Error reading config params");}
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:11-20");
		Q_ASSERT(par.value > 0 and par.value < 500);
		for(int i=11;i<=20; i++)
			tagsSizeMap.insert( i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:21-30");
		Q_ASSERT(par.value > 0 and par.value < 500);
		for(int i=21;i<=30; i++)
			tagsSizeMap.insert( i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}
	
	//DONE
	
	//Default value for IDs not defined before

	return true;
}

void SpecificWorker::compute()
{

	static int frame = 0;
	double last_t = tic();
	
	RoboCompCamera::imgType img;
	if( INPUTIFACE == Camera)
	{
		//For cameras
		try
		{
			camera_proxy->getYImage(0,img, cState, bState);
			memcpy(image_gray.data, &img[0], 640*480*sizeof(uchar));
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
		qFatal("Support for RGBDBus is not implemented yet!");
	}
	else
		qFatal("Input device not defined. Please specify one in the config file");

	
	
	vector< ::AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray); 

	// print out each detection
	cout << detections.size() << " tags detected:" << endl;

	print_detection(detections);
// 
	if (m_draw) 
	{
		for (uint i=0; i<detections.size(); i++) 
		{
			detections[i].draw(image_gray);
		}
		//imshow("AprilTags", image_gray); // OpenCV call
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
	
	for(uint32_t i=0; i<detections.size(); i++)
	{
		::AprilTags::TagDetection detection = detections[i];  //PROBAR CON REFERENCIA PARA EVITAR LA COPIA
		
		cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ")";

		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		
		//detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);
		
		///SIN PROBAR PERO DEBERIA IR. SI NO ENCUNETRA EL ID METE m_tagSize
		const float ss = tagsSizeMap.value(detection.id, m_tagSize);
		qDebug() << "ss" << ss;
		detection.getRelativeTranslationRotation(ss, m_fx, m_fy, m_px, m_py, translation, rotation);
		QVec T(3);
		T(0) = -translation(1);
		T(1) =  translation(2);
		T(2) =  translation(0);
		
		Eigen::Matrix3d F;
		F << 1, 0,  0,	0,  -1,  0,	0,  0,  1;
		Eigen::Matrix3d fixed_rot = F*rotation;
		
		double rx, ry, rz;
		rotationFromMatrix(fixed_rot, rx, ry, rz);

		cout << "  distance=" << T.norm2()
			 << ", x=" << T(0)
			 << ", y=" << T(1)
			 << ", z=" << T(2)
			 << ", rx=" << rx
			 << ", ry=" << ry
			 << ", rz=" << rz
			 << endl;

		// Also note that for SLAM/multi-view application it is better to
		// use reprojection error of corner points, because the noise in
		// this relative pose is very non-Gaussian; see iSAM source code
		// for suitable factors.
		// fill in tag to send
		RoboCompAprilTags::tag t;
		RoboCompGetAprilTags::marca mar;

		t.id=detection.id;
		///ASDFASDFAAAAAAAAAAAAAAAAAAAAAAA
		t.tx=T(0);
		t.ty=T(1);
		t.tz=T(2);
		//Change the x,y,z rotation values to match robocomp's way
		t.rx=rx;
		t.ry=ry;
		t.rz=rz;

		memcpy(&mar, &t, sizeof(RoboCompGetAprilTags::marca));
		mutex->lock();
			detections2send[i]=t;
			listaDeMarcas[i]=mar;
		mutex->unlock();
	}
		
	try
	{
		apriltags->newAprilTag(detections2send);
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

void SpecificWorker::rotationFromMatrix(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz) 
{
	QMat v(3,3);
	for (uint32_t f=0; f<3; f++)
	{
		for (uint32_t c=0; c<3; c++)
		{
			v(f,c) = R(f,c);
		}
	}
	QVec ret = v.extractAnglesR_min();
	rx = ret(0)+M_PIl;
	while (rx >= M_PIl) rx -= 2.*M_PIl;
	while (rx < -M_PIl) rx += 2.*M_PIl;
	ry = -ret(1);
	rz = ret(2);

/*
	yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
	roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
*/
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
