#ifndef VIEWER_H
#define VIEWER_H

#include <QWidget>

#include <pcl/io/openni_grabber.h>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include "innermodelManager.h"

#include <string>

using namespace std;

typedef pcl::PointXYZRGBA PointT;

class Viewer: public QObject
{
Q_OBJECT

	QTimer timer;
	OsgView *world3D;
	InnerModelViewer *innermodelviewer;
	InnerModel *innermodel;
	InnerModelManager *innermodelmanager;
	QMutex innermodelMutex;

//   osg::ref_ptr<osg::Image> osgImage;
  
public:
	Viewer(string innermodelMap, QWidget *p);
	~Viewer();
  
  inline void setHFOV(float HFOV) { world3D->autoResize(HFOV); }

	void setPointCloud(pcl::PointCloud<PointT>::Ptr cloud);
	void resizeEvent(QResizeEvent * event);

	void setPose(std::string item,  QVec t,  QVec r,  QVec s);
	void setScale(std::string item, float scaleX,float scaleY, float scaleZ);

	void showImage( int32_t width, int32_t height, uint8_t *rgb_image);

public slots:
	void update();

};
#endif