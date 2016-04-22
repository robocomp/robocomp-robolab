/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
	innerViewer        = NULL;
	osgView            = new OsgView(this->widget);

	graph = new ConnectivityGraph("/home/robocomp/robocomp/components/robocomp-ursus/components/ikGraphGenerator/ursusRt.ikg");


	target_index = 0;
	myVertices = graph->vertices;
	std::random_shuffle ( myVertices.begin(), myVertices.end() );
	
	show();
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
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}

	InnerModelNode *nodeParent = innerModel->getNode("root");
	if( innerModel->getNode("target") == NULL)
	{
		InnerModelTransform *node = innerModel->newTransform("target", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
	}

	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);
		delete innerViewer;
	}
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), false);

	QMutexLocker ml(mutex);
	timer.start(Period);
	
	IMVCamera cam = innerViewer->cameras["rgbd"];
	const int width = cam.RGBDNode->width;
	const int height = cam.RGBDNode->height;
	if (color.size() != (uint)width*height)
	{
		color.resize ( width*height );
		depth.resize ( width*height );
		points.resize ( width*height );
	}


	return true;
}

void SpecificWorker::compute()
{
	static float v = 0;
	innerModel->updateTransformValues("rgbd_transform", 0,0,0,  v+=0.5,0,0);
	innerViewer->update();
		osgView->autoResize();
		osgView->frame();
	renderAndGenerateImages();
}


void SpecificWorker::renderAndGenerateImages()
{
	if (innerViewer)
	{
// 		innerViewer->update();
// 		osgView->autoResize();
// 		osgView->frame();
		IMVCamera cam = innerViewer->cameras["rgbd"];
		cam.viewerCamera->frame();

		const int width = cam.RGBDNode->width;
		const int height = cam.RGBDNode->height;
		const float focal = float(cam.RGBDNode->focal);
		double fovy, aspectRatio, Zn, Zf;
		
		RTMat rt = innerModel->getTransformationMatrix("root", "rgbd");
		innerViewer->cameras["rgbd"].viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));
		cam.viewerCamera->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, Zn, Zf);


#ifdef GENERATEUSELESSIMAGESTOCHECKTHERENDERINGISWORKING
	QImage img = QImage(width, height, QImage::Format_RGB888);
#endif

		const unsigned char *rgb = cam.rgb->data();
		const float *d = (float *)cam.d->data();
		for (int i=0; i<height; ++i)
		{
			for (int j=0; j<width; ++j)
			{
				const int index  = j + i*width;
				const int indexI = j + (height-1-i)*width;
				color[index].red   = rgb[3*indexI+0];
				color[index].green = rgb[3*indexI+1];
				color[index].blue  = rgb[3*indexI+2];
				if (d[indexI] <= 1.)
				{
					depth[index] = Zn*Zf / ( Zf - d[indexI]* ( Zf-Zn ) );
				}
				else
				{
					depth[i] = NAN;
				}
				points[index].x = depth[index] * (float(j)    - (width/2.)) / focal;
				points[index].y = depth[index] * ((height/2.) -   float(i)) / focal;
				points[index].z = depth[index];
				points[index].w = 1.;
				

#ifdef GENERATEUSELESSIMAGESTOCHECKTHERENDERINGISWORKING
				img.setPixel(j, i, depth[index]*50);
#endif
			}
		}
#ifdef GENERATEUSELESSIMAGESTOCHECKTHERENDERINGISWORKING
		static int frame = 0;
		img.save(QString("depth")+QString("%1").arg(frame++, 5, 10, QChar('0'))+".png");
		sleep(3);
#endif
		printf("rendered!\n");
	}
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{

}






