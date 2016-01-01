#ifndef LMAP_H
#define LMAP_H

#include <opencv2/opencv.hpp>

#include <innermodel/innermodel.h>

#include <Laser.h>


class LMap
{

public:
	LMap(float side_, int32_t bins_, float laserRange_);
	void update(RoboCompLaser::TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString robotID, QString laserID);
	RoboCompLaser::TLaserData getData();

private:
	int32_t bins;
	float side;
	float laserRange;
	cv::Mat map;
	cv::Mat mapM;
	QTime lastForget;
	
};




#endif


