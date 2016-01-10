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
	LMap(float side_, int32_t bins_, float laserRange_, const QString &movableRootID_, const QString &virtualLaserID_, InnerModel *innerModel_, const QString &robotID, const float robotRadius);

	void update_timeAndPositionIssues(QString actualLaserID);
	void update_include_laser(TLaserData *laserData, QString actualLaserID);
	void update_include_rgbd(RoboCompRGBD::PointSeq *points, QString rgbdID);
	void update_done(QString actualLaserID, float minDist);

	void getLaserData(TLaserData *laserData, int32_t bins, float maxLength);

private:
	int32_t bins;
	float side;
	float laserRange;
	cv::Mat mapLaser;
	cv::Mat mapRGBDs;
	cv::Mat mapBlend;
	cv::Mat mapThreshold;
	QTime lastForgetSubtractLaser;
	QTime lastForgetSubtractRGBD;
	QTime lastForgetAdd;
	InnerModel *innerModel;
	QString movableRootID;
	QString virtualLaserID;
	QString robotID;
	float robotRadius;

private:
	inline int32_t angle2bin(double ang, const int bins);
	inline QVec fromReferenceLaserToImageCoordinates(const float dist, const float angle, const QString &reference);
	inline QVec fromReferenceToImageCoordinates(const QVec &point, const QString &reference);
	inline QVec fromImageToReference(float xi, float zi, const QString &reference);
	inline void fromImageToVirtualLaser(float xi, float zi, int32_t laserBins, float &dist, int32_t &bin);
	inline void addToLaserCoordinates(const int x, const int z);
	inline void addToRGBDsCoordinates(const int x, const int z);
};




#endif


