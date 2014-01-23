#include "innermodelManager.h"

InnerModelManager::InnerModelManager(InnerModel *innermodel, InnerModelViewer *imv)
{
	this->innerModel=innermodel;
	this->imv=imv;
}

void InnerModelManager::setPointCloudData(const std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	QString m = QString("setPointCloudData");

	/// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
	IMVPointCloud *pcNode = imv->pointCloudsHash[QString::fromStdString(id)];

	int points = cloud->size();
	pcNode->points->resize(points);
	pcNode->colors->resize(points);
	pcNode->setPointSize(3);
	int i = 0;
	for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin(); it != cloud->end(); it++ )
	{
		if (!pcl_isnan(it->x)&&!pcl_isnan(it->y)&&!pcl_isnan(it->z))
		{
			pcNode->points->operator[](i) = QVecToOSGVec(QVec::vec3(it->x, it->y, it->z));
			pcNode->colors->operator[](i) = osg::Vec4f(float(it->r)/255, float(it->g)/255, float(it->b)/255, 1.f);
		}
		i++;
	}
	pcNode->update();
	imv->update();
}

void InnerModelManager::setImageOnPlane(int32_t width, int32_t height, uint8_t  *rgb_image, const std::string id)
{
	QString m = QString("setImageOnPlane");
	std::cout<<"InnerModelManager::setImageOnPlane: "<<std::endl;

	/// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
	IMVPlane *pcNode = imv->planesHash[QString::fromStdString(id)];
	pcNode->updateBuffer(rgb_image, width, height);

	pcNode->performUpdate();
	imv->update();
}

void InnerModelManager::setPose(std::string item,  QVec t,  QVec r,  QVec s)
{
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setPoseFromParent()";

	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(QString::fromStdString(item),m));
	checkOperationInvalidNode(aux,m + qItem +"can't be use as base because it's not a InnerModelTransform node.");

	innerModel->updateTransformValues(qItem, t(0), t(1), t(2), r(0), r(1) , r(2));

	imv->update();
}

void InnerModelManager::setScale(std::string item, float scaleX,float scaleY, float scaleZ)
{
	//QMutexLocker locker (mutex);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setScale()";

	InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(getNode(QString::fromStdString(item),m));
	checkOperationInvalidNode(aux,m + qItem +"can't be use as base because it's not a InnerModelMesh node.");

	aux->setScale(scaleX, scaleY, scaleZ);
	imv->update();
}

void InnerModelManager::checkOperationInvalidNode(InnerModelNode *node,QString msg)
{
	if (node==NULL)
	{
		std::ostringstream oss;
		oss <<msg.toStdString()<<" error: Node " << node->id.toStdString()<<" is not a Transform";
		throw "dedede";
	}
}

InnerModelNode *InnerModelManager::getNode(const QString &id, const QString &msg)
{
	InnerModelNode *node = innerModel->getNode(id);
	if (node==NULL)
	{
		std::ostringstream oss;
		oss << msg.toStdString() << " error: Node " << id.toStdString() << " does not exist.";
		throw "no such node";
	}
	else
	{
		return node;
	}
	return NULL;
}
