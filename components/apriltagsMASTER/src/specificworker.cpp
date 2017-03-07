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
#include<opencv2/highgui/highgui.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx), m_tagDetector(NULL), m_tagCodes(::AprilTags::tagCodes16h5), m_draw(true)
{
	innermodel = NULL;
	//cv::namedWindow("deo");
	worker_params_mutex = new QMutex();
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	aux.type = "float";
	aux.value = "0";
	worker_params["frameRate"] = aux;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//Default value
	m_width = 640;
	m_height = 480;

	try
	{
		INPUTIFACE = Camera;
		RoboCompCommonBehavior::Parameter par = params.at("InputInterface");
		if (par.value == "RGBD")
		{
			printf("INTERFACE RGBD selected\n");
			INPUTIFACE = RGBD;
		}
		else if ( par.value == "RGBDBus")
		{
			printf("INTERFACE RGBDBus selected\n");
			INPUTIFACE = RGBDBus;
		}
		else if ( par.value == "Camera")
		{
			printf("INTERFACE Camera selected\n");
			INPUTIFACE = Camera;
			try
			{
				RoboCompCommonBehavior::Parameter par = params.at("CameraResolution");
				if ( par.value == "320x240" )
				{
					m_width = 320;
					m_height = 240;
				}
				if ( par.value == "640x480" )
				{
					m_width = 640;
					m_height = 480;
				}
			}
			catch(std::exception e)
			{}
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

	m_px = m_width/2;
	m_py = m_height/2;

	printf("w:%d   h:%d\n", m_width, m_height);
	image_gray.create(m_height,m_width,CV_8UC1);
	image_color.create(m_height,m_width,CV_8UC3);
	try
	{
		innermodel = new InnerModel(innermodel_path);
	}
	catch (QString &s)
	{
		printf("error reading iner model %s\n", s.toStdString().c_str());
	}
	m_fx = innermodel->getCamera("rgbd")->getFocal();
	m_fy = innermodel->getCamera("rgbd")->getFocal();

	qDebug() << QString::fromStdString(innermodel_path) << " " << QString::fromStdString(camera_name);
	qDebug() << "FOCAL LENGHT:" << innermodel->getCamera("rgbd")->getFocal();

	//Reading id sets size to create a map

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:0-10");
		qDebug() << "ID:0-10" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=0;i<=10; i++)
			tagsSizeMap.insert( i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) {  qFatal("Error reading config params");}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:11-20");
		qDebug() << "ID:11-20" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=11;i<=20; i++)
			tagsSizeMap.insert(i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:21-100");
		qDebug() << "ID:21-100" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=21;i<=100; i++)
			tagsSizeMap.insert(i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}


	// Default value for IDs not defined before
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("AprilTagsSize");
		qDebug() << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		m_tagSize = QString::fromStdString(par.value).toFloat();
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}

	timer.start(100);
	
	return true;
}

void SpecificWorker::compute()
{
	static QTime reloj = QTime::currentTime();
	if (innermodel == NULL) return;
	
	printf("---------------------\n"); 

	static int frame = 0;
	static QTime lastTime = QTime::currentTime();

	//printf("FOCAL: %fx%f   sizes:(%f)[ ", float(m_fx), float(m_fy), float(m_tagSize));
	// 	for (QMap<int, float>::iterator it = tagsSizeMap.begin(); it!=tagsSizeMap.end(); it++)
	// 	{
	// 		printf("%d:%f ", it.key(), it.value());
	// 	}
	printf("]\n");

	RoboCompCamera::imgType img;
	if( INPUTIFACE == Camera)
	{
		//For cameras
		try
		{
			camera_proxy->getYImage(0,img, cState, bState);
			memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
			searchTags(image_gray);
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
			RoboCompRGBD::ColorSeq colorseq;
			RoboCompRGBD::DepthSeq depthseq;
			rgbd_proxy->getRGB(colorseq, hState, bState);
			memcpy(image_color.data , &colorseq[0], m_width*m_height*3);
			cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
			searchTags(image_gray);	
			//imshow("deo", image_gray);
			//cv::waitKey(10);
		}
		catch(const Ice::Exception &e)
		{
			std::cout << "Error reading form RGBD " << e << std::endl;
		}
	}
	else if( INPUTIFACE == RGBDBus)
	{
		RoboCompRGBDBus::ImageMap images;
		CameraList cameraList;
		cameraList.push_back(std::string("default"));
		rgbdbus_proxy->getImages(cameraList, images);
		for (auto i : images)
		{
			memcpy(image_color.data , &i.second.colorImage[0], m_width*m_height*3);
			//cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
			searchTags(image_gray);
		}
	}
	else
	{
		qFatal("Input device not defined. Please specify one in the config file");
	}

	// print out the frame rate at which image frames are being processed
	frame++;
	if (frame >= 30)
	{
		cout << "*********************************************************" <<  1000.*frame/lastTime.elapsed()  << " fps" << endl;
		frame = 0;
	}
	worker_params_mutex->lock();
		//save framerate in params
		worker_params["frameRate"].value = std::to_string(reloj.restart()/1000.f);
	worker_params_mutex->unlock();
	
}

void SpecificWorker::searchTags(const cv::Mat &image_gray)
{
	vector< ::AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

	cout << detections.size() << " tags detected:" << endl;
	print_detection(detections);

}
void SpecificWorker::print_detection(vector< ::AprilTags::TagDetection> detections)
{
	detections2send.resize(detections.size());
	listaDeMarcas.resize(detections.size());

	for(uint32_t i=0; i<detections.size(); i++)
	{
		::AprilTags::TagDetection detection = detections[i];  //PROBAR CON REFERENCIA PARA EVITAR LA COPIA

 		cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ")";

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;

		///SIN PROBAR PERO DEBERIA IR. SI NO ENCUNETRA EL ID METE m_tagSize
		const float ss = tagsSizeMap.value(detection.id, m_tagSize);

		detection.getRelativeTranslationRotation(ss, m_fx, m_fy, m_px, m_py, translation, rotation);
		QVec T(3);
		T(0) = -translation(1);//*0.65;
		T(1) =  translation(2);//*0.65;
		T(2) =  translation(0);//*0.65;

		Eigen::Matrix3d F;
		F << 1, 0,  0,	0,  -1,  0,	0,  0,  1;
		Eigen::Matrix3d fixed_rot = F*rotation;

		double rx, ry, rz;
		rotationFromMatrix(fixed_rot, rx, ry, rz);

		cout << m_fx << "  " << m_fy << endl;
		cout << "  distance=" << T.norm2() << ", x=" << T(0) << ", y=" << T(1) << ", z=" << T(2) << ", rx=" << rx << ", ry=" << ry << ", rz=" << rz << endl;
		//rDebug2(("TAG: Distance=%d x=%d y=%d z=%d rx=%d ry=%d rz=%d")%T.norm2()%T(0)%T(1)%T(2)%rx%ry%rz);
		
		RoboCompAprilTags::tag t;
		RoboCompGetAprilTags::marca mar;

		t.id=detection.id;
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

	if (detections2send.size() > 0)
	{
		try
		{
			apriltags_proxy->newAprilTag(detections2send);
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
		}
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
RoboCompCommonBehavior::ParameterList SpecificWorker::getWorkerParams()
{
	QMutexLocker locker(worker_params_mutex);
	return worker_params;
}



