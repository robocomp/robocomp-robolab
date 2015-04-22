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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	//namedWindow("img",1);
	timer.start(100);

	return true;
}

void SpecificWorker::compute()
{
	try
	{
		vector<string> names;
		names.push_back("default");
		RoboCompRGBDBus::ImageMap imgs;
		rgbdbus_proxy->getImages(names, imgs);
		Mat frame(imgs["default"].camera.colorHeight, imgs["default"].camera.colorWidth, CV_8UC3,  &(imgs["default"].colorImage)[0]);
		drawQtImage((const uchar *)(&(imgs["default"].colorImage)[0]), imgs["default"].camera.colorWidth, imgs["default"].camera.colorHeight, QImage::Format_RGB888, label);
		
		/// Apply Laplace function
		Mat gray, dst, abs_dst;
		cvtColor( frame, gray, CV_RGB2GRAY );
		Laplacian( gray, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
		convertScaleAbs( dst, abs_dst );
		drawQtImage( (const uchar *)(abs_dst.data), imgs["default"].camera.colorWidth, imgs["default"].camera.colorHeight, QImage::Format_Indexed8, label_edges);
		
		printFPS();
		//imshow("img", frame);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}

void SpecificWorker::printFPS()
{
	static QTime reloj = QTime::currentTime();
	static int cont = 0;
	
	uint lag = reloj.elapsed();
	if( lag > 1000) 
	{
		lcdNumber->display( 1000.f*cont/lag);
		reloj.restart();
		cont = 0;
	}
	cont++;
}


void SpecificWorker::drawQtImage( const uchar *data, int width, int height, QImage::Format format, QLabel *label)
{
	QImage img = QImage(data, width, height, format);
	label->setPixmap(QPixmap::fromImage(img));
	label->resize(label->pixmap()->size());
}


