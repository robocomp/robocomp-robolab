#ifndef LMAP_H
#define LMAP_H

#include <opencv2/opencv.hpp>

#include <innermodel/innermodel.h>

#include <Laser.h>
#include <RGBD.h>

using namespace RoboCompLaser;

class LMap
{

public:
	LMap(float side_, int32_t bins_, float laserRange_, InnerModel *innerModel_);

	void update_timeAndPositionIssues(QString movableRootID, QString virtualLaserID, QString actualLaserID);
	void update_include_laser(TLaserData *laserData, QString movableRootID, QString virtualLaserID, QString actualLaserID);
	void update_include_rgbd(RoboCompRGBD::PointSeq *points, QString movableRootID, QString virtualLaserID, QString rgbdID);
	void update_done(QString movableRootID, QString virtualLaserID, QString actualLaserID, float minDist);

	void getLaserData(TLaserData *laserData, QString movableRootID, QString virtualLaserID, int32_t bins, float maxLength);

private:
	int32_t bins;
	float side;
	float laserRange;
	cv::Mat map;
	cv::Mat mapThreshold;
	QTime lastForgetSubtract;
	QTime lastForgetAdd;
	InnerModel *innerModel;

private:
	inline int32_t angle2bin(double ang, const int bins);
	inline QVec fromReferenceLaserToImageCoordinates(const float dist, const float angle, const QString &reference, const QString &movableRootID);
	inline QVec fromReferenceToImageCoordinates(const QVec &point, const QString &reference, const QString &movableRootID);
	inline void addToCoordinates(const int x, const int z);
};




#endif


