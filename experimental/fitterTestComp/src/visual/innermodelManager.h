#ifndef INNERMODELMANAGER_H
#define INNERMODELMANAGER_H

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

class InnerModelManager
{
  InnerModel *innerModel;
  InnerModelViewer *imv;
  
  void checkOperationInvalidNode(InnerModelNode *node,QString msg);
  InnerModelNode *getNode(const QString &id, const QString &msg);
public:
  InnerModelManager(InnerModel *innermodel, InnerModelViewer *imv);
  void setPointCloudData(const std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void setImageOnPlane(int32_t width, int32_t height, uint8_t  *rgb_image, const std::string id);
  void setPose(std::string item,  QVec t,  QVec r,  QVec s);
  void setScale(std::string item, float scaleX,float scaleY, float scaleZ);

};

#endif