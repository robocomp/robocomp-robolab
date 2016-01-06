#ifndef LMAP_H
#define LMAP_H

#include <opencv2/opencv.hpp>

#include <innermodel/innermodel.h>

#include <Laser.h>

using namespace RoboCompLaser;

class LMap
{

public:
	LMap(float side_, int32_t bins_, float laserRange_);

	void update_timeAndPositionIssues(InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID);
	void update_laser(TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID);
	void update_done(InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID, float minDist);

	void getLaserData(TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString virtualLaserID, int32_t bins, float laserFOV, float maxLength);

private:
	int32_t bins;
	float side;
	float laserRange;
	cv::Mat map;
	cv::Mat mapThreshold;
	QTime lastForgetSubtract;
	QTime lastForgetAdd;

private:
	inline int32_t angle2bin(double ang, const float laserFOV, const int bins)
	{
		while (ang>M_PI)  ang -= 2.*M_PI;
		while (ang<-M_PI) ang += 2.*M_PI;

		double ret;
		ang += laserFOV/2.;
		ret = (ang * bins) / laserFOV;
		return int32_t(ret);
	}
};




#endif


