#include "viewer.h"

Viewer::Viewer(string innermodelMap, QWidget *p)
{
	innermodel = new InnerModel(innermodelMap);

	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	world3D = new OsgView(p);
	world3D->init();

	innermodelviewer = new InnerModelViewer(innermodel, "root", world3D->getRootGroup());

	world3D->getRootGroup()->addChild(innermodelviewer);
	world3D->show();
	world3D->setHomePosition(osg::Vec3(0,0,0),osg::Vec3(0.f,0.,-4000.),osg::Vec3(0.0f,1000.f,0.0f), false);

	innermodelmanager = new InnerModelManager(innermodel, innermodelviewer);
}


Viewer::~Viewer()
{
	delete(world3D);
	delete(innermodel);
	delete(innermodelviewer);
}

void Viewer::showImage( int32_t width, int32_t height, uint8_t  *rgb_image)
{
		innermodelMutex.lock();
		innermodelviewer->planesHash["back"]->updateBuffer(rgb_image, width, height);
		innermodelMutex.unlock();
}

void Viewer::setPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
	innermodelMutex.lock();
	innermodelmanager->setPointCloudData("cloud", cloud);
	innermodelMutex.unlock();
}

void Viewer::setPose(std::string item,  QVec t,  QVec r,  QVec s)
{
	innermodelMutex.lock();
	innermodelmanager->setPose(item, t, r, s );
	innermodelMutex.unlock();
	world3D->update();
}

void Viewer::setScale(std::string item, float scaleX,float scaleY, float scaleZ)
{
	innermodelMutex.lock();
	innermodelmanager->setScale(item, scaleX, scaleY, scaleZ);
	innermodelMutex.unlock();
	world3D->update();
}

void Viewer::update()
{
	innermodelviewer->update();
	world3D->update();
}

void Viewer::resizeEvent(QResizeEvent * event)
{
	world3D->autoResize();
}
