#include <math.h>
#include "map.h"

using namespace std;

LMap::LMap(float side_, int32_t bins_, float laserRange_, const QString &movableRootID_, const QString &virtualLaserID_, InnerModel *innerModel_, const QString &robotID_, const float robotRadius_)
{
	Q_ASSERT(0.5*side_ > 1.25 * laserRange);
	side = side_;
	bins = bins_;
	innerModel = innerModel_;
	laserRange = laserRange_;
	robotID = robotID_;
	robotRadius = robotRadius_;
	// TODO It would be nice to create the InnerModel node in case it doesn't exist (the one for movableRootID) TODO
	movableRootID = movableRootID_;
	virtualLaserID = virtualLaserID_;
	lastForgetSubtractLaser = QTime::currentTime();
	lastForgetSubtractRGBD = QTime::currentTime();
	lastForgetAdd = QTime::currentTime();

	mapLaser = cv::Mat(bins, bins, CV_8UC1, cv::Scalar(128));
	mapRGBDs = cv::Mat(bins, bins, CV_8UC1, cv::Scalar(128));
	mapBlend = cv::Mat(bins, bins, CV_8UC1, cv::Scalar(128));
//	cv::namedWindow("mapLaser", cv::WINDOW_AUTOSIZE);
//	cv::namedWindow("mapRGBDs", cv::WINDOW_AUTOSIZE);
//	cv::namedWindow("mapBlend", cv::WINDOW_AUTOSIZE);
//	cv::namedWindow("mapThreshold", cv::WINDOW_AUTOSIZE);
}


cv::Mat offsetImageWithPadding(const cv::Mat& originalImage, int offsetX, int offsetY)
{
	auto padded = cv::Mat(originalImage.rows + 2 * abs(offsetY), originalImage.cols + 2 * abs(offsetX), CV_8UC1, cv::Scalar(128));
	originalImage.copyTo(padded(cv::Rect(abs(offsetX), abs(offsetY), originalImage.cols, originalImage.rows)));
	return cv::Mat(padded, cv::Rect(abs(offsetX) + offsetX, abs(offsetY) + offsetY, originalImage.cols, originalImage.rows));
}


void LMap::update_timeAndPositionIssues(QString actualLaserID)
{
	// First, if virtualLaserID is far from the movable root, move the movableRootID reference.
	const QVec relPose = innerModel->transform(movableRootID, virtualLaserID);
	float distFromReference = innerModel->transform(movableRootID, virtualLaserID).norm2();
	float margin = 0.5*side-laserRange;
	static bool first = true;
	if (distFromReference >= margin or first)
	{
		first = false;
		QVec relPosePixels = relPose.operator*(float(bins)/float(side));
		relPosePixels.print("relPosePixels");
		mapLaser = offsetImageWithPadding(mapLaser, relPosePixels(0), -relPosePixels(2));
		mapRGBDs = offsetImageWithPadding(mapRGBDs, relPosePixels(0), -relPosePixels(2));
		relPosePixels.print("relPosePixels");
		QString mrID_parent = innerModel->getParentIdentifier(movableRootID);
		QVec pFromMRIDp = innerModel->transform(mrID_parent, virtualLaserID);
		innerModel->updateTransformValues(movableRootID, pFromMRIDp(0), pFromMRIDp(1), pFromMRIDp(2), 0,0,0);
		printf("moving movableRootID %f\n", distFromReference);
	}

	// Decrease the obstacle certainty
	const float forgetRateSubtractLaser = 7;
	const float forgetSubtractLaser = float(forgetRateSubtractLaser*lastForgetSubtractLaser.elapsed())/1000.;
	if (forgetSubtractLaser > 2)
	{
		lastForgetSubtractLaser = QTime::currentTime();
		for (int x=0; x<bins; x++)
		{
			for (int z=0; z<bins; z++)
			{
				int64_t t = mapLaser.at<uchar>(z, x);
				if (t > 128)
				{
					t -= forgetSubtractLaser;
					if (t<128) t = 128;
					mapLaser.at<uchar>(z, x) = t;
				}
			}
		}
	}
	// Decrease the obstacle certainty
	const float forgetRateSubtractRGBD = 5;
	const float forgetSubtractRGBD = float(forgetRateSubtractRGBD*lastForgetSubtractRGBD.elapsed())/1000.;
	if (forgetSubtractRGBD > 2)
	{
		lastForgetSubtractRGBD = QTime::currentTime();
		for (int x=0; x<bins; x++)
		{
			for (int z=0; z<bins; z++)
			{
				int64_t t = mapRGBDs.at<uchar>(z, x);
				if (t > 128)
				{
					t -= forgetSubtractRGBD;
					if (t<128) t = 128;
					mapRGBDs.at<uchar>(z, x) = t;
				}
			}
		}
	}
	// Increase the obstacle certainty
	const float forgetRateAdd = 7;
	const float forgetAdd = float(forgetRateAdd*lastForgetAdd.elapsed())/1000.;
	if (forgetAdd > 5)
	{
		lastForgetAdd = QTime::currentTime();
		for (int x=0; x<bins; x++)
		{
			for (int z=0; z<bins; z++)
			{
				int64_t t;
				t = mapLaser.at<uchar>(z, x);
				if (t < 128)
				{
					t += forgetAdd;
					if (t>128) t = 128;
					mapLaser.at<uchar>(z, x) = t;
				}
				t = mapRGBDs.at<uchar>(z, x);
				if (t < 128)
				{
					t += forgetAdd;
					if (t>128) t = 128;
					mapRGBDs.at<uchar>(z, x) = t;
				}
			}
		}
	}
}


void LMap::update_include_laser(RoboCompLaser::TLaserData *laserData, QString actualLaserID)
{
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord  = fromReferenceLaserToImageCoordinates(datum.dist-(6.*side/bins), datum.angle, actualLaserID);
		int xImageCoord = mapCoord(0);
		int zImageCoord = mapCoord(2);
		// Empty a line
		const QVec mapCoordZ = fromReferenceLaserToImageCoordinates(                        0, datum.angle, actualLaserID);
		int xImageCoordL = mapCoordZ(0);
		int zImageCoordL = mapCoordZ(2);
		cv::line(mapLaser, cv::Point(xImageCoord, zImageCoord), cv::Point(xImageCoordL, zImageCoordL), cv::Scalar(0), 5);
	}
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord = fromReferenceLaserToImageCoordinates(datum.dist, datum.angle, actualLaserID);
		// Draw obstacle
		if (datum.dist < laserRange and mapCoord(0)>2 and mapCoord(0)<bins-2 and mapCoord(2)>2 and mapCoord(2)<bins-2)
		{
			for (auto i : std::vector<int>{-1, 0, 1})
			{
				addToLaserCoordinates(mapCoord(0)+i, mapCoord(2));
				addToLaserCoordinates(mapCoord(0), mapCoord(2)+i);
			}
		}
	}
}

void LMap::update_include_rgbd(RoboCompRGBD::PointSeq *points, QString rgbdID)
{
	int minHeight = 100;
	int maxHeight = 1900;
	int minHeightNeg = -200;

	RTMat TRr = innerModel->getTransformationMatrix("robot", rgbdID); // WARNING This was used because it could be fater than calling transform multiple times
	const uint32_t rgbd_size = points->size();
	uint32_t pw = 640;
	uint32_t ph = 480;
	uint32_t stepW = 10;
	uint32_t stepH = 16;
	if (points->size() == 320*240) { pw=320; ph=240; stepW=7; stepH=10; }
	if (points->size() == 160*120) { pw=160; ph=120; stepW=4; stepH=6;  }
	if (points->size() == 80*60)   { pw= 80; ph= 60; stepW=3; stepH=2;  }
	#pragma omp parallel for
	for (uint32_t rr=0; rr<ph; rr+=stepH)
	{
		for (uint32_t cc=0; cc<pw; cc+=stepW)
		{
			uint32_t ioi = rr*pw+cc;
			if (ioi<rgbd_size)
			{
				if (std::isnan(points->operator[](ioi).x) or std::isnan(points->operator[](ioi).y) or std::isnan(points->operator[](ioi).z))
					continue;
				if (points->operator[](ioi).z<0)
					continue;
				const QVec pRobot = (TRr * QVec::vec4(points->operator[](ioi).x, points->operator[](ioi).y, points->operator[](ioi).z, 1)).fromHomogeneousCoordinates();
				if (not ( (pRobot(1)>=minHeight and pRobot(1)<=maxHeight) or (pRobot(1)<minHeightNeg) ))
					continue;
				const QVec mapCoord = fromReferenceToImageCoordinates(pRobot, "robot");
				if (mapCoord(0)<=2 or mapCoord(0)>=bins-2 or mapCoord(2)<=2 or mapCoord(2)>=bins-2)
					continue;
				addToRGBDsCoordinates(mapCoord(0), mapCoord(2));
			}
		}
	}
}

void LMap::update_done(QString actualLaserID, float minDist)
{
	// Empty the robot's position
	const QVec mapCoord = fromReferenceToImageCoordinates(QVec::vec3(0,0,0), robotID);
	int xImageCoord = mapCoord(0);
	int zImageCoord = mapCoord(2);

	cv::circle(mapLaser, cv::Point(xImageCoord, zImageCoord), robotRadius*float(bins)/float(side), cv::Scalar(0), -1, 8, 0);
	cv::circle(mapRGBDs, cv::Point(xImageCoord, zImageCoord), robotRadius*float(bins)/float(side), cv::Scalar(0), -1, 8, 0);

	cv::addWeighted(mapLaser, 0.45, mapRGBDs, 0.55, 0, mapBlend);
	

	// Thresholding
	cv::threshold(mapBlend, mapThreshold, 127, 128, cv::THRESH_BINARY);

	// Dilate
	int dilation_size = 1;
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size( 2*dilation_size+1, 2*dilation_size+1), cv::Point( dilation_size, dilation_size ) );
	cv::dilate(mapThreshold, mapThreshold, element);

	// Show
//	cv::imshow("mapLaser", mapLaser);
//	cv::imshow("mapRGBDs", mapRGBDs);
//	cv::imshow("mapBlend", mapBlend);
//	cv::imshow("mapThreshold", mapThreshold);
//	if (cv::waitKey(3) == 27)
//		exit(0);
}


void LMap::getLaserData(RoboCompLaser::TLaserData *laserData, int32_t laserBins, float maxLength)
{
	/// Clear laser measurement
	for (int32_t i=0; i<laserBins; ++i)
	{
		(*laserData)[i].dist = maxLength;
	}

// 	const QVec lmapCoord = fromReferenceToImageCoordinates(QVec::vec3(0,0,0), virtualLaserID);
// 	int radius = (maxLength*float(bins))/float(side);
	for (int xi=0; xi<bins; xi+=2)
	{
		for (int zi=0; zi<bins; zi+=2)
		{
// 			if (QVec::vec3(lmapCoord(0)-xi, lmapCoord(2)-zi).norm2()<=radius);
			{
				if (mapThreshold.at<uchar>(zi, xi) > 0)
				{
					float dist;
					int32_t bin;
					fromImageToVirtualLaser(xi, zi, laserBins, dist, bin);
					if (dist < (*laserData)[bin].dist)
					{
						(*laserData)[bin].dist = dist;
					}
				}
			}
		}
	}
}

inline QVec LMap::fromImageToReference(float xi, float zi, const QString &reference)
{
	QVec mapCoordInMovableRoot = QVec::vec3(xi-0.5*bins, 0, 0.5*bins-zi);
	mapCoordInMovableRoot = mapCoordInMovableRoot.operator*(float(side)/float(bins));
	return innerModel->transform(reference, mapCoordInMovableRoot, movableRootID);
}

inline void LMap::fromImageToVirtualLaser(float xi, float zi, int laserBins, float &dist, int32_t &bin)
{
	QVec mapCoordInLaser = fromImageToReference(xi, zi, virtualLaserID);
	float angle = atan2(mapCoordInLaser(0), mapCoordInLaser(2));
	dist = mapCoordInLaser.norm2();
	bin = angle2bin(angle, laserBins);
}

inline int32_t LMap::angle2bin(double ang, const int bins)
{
	while (ang>M_PI)  ang -= 2.*M_PI;
	while (ang<-M_PI) ang += 2.*M_PI;

	double ret;
	ang += M_PIl*2./2.;
	ret = (ang * bins) / (2.*M_PIl);
	return int32_t(ret);
}

inline QVec LMap::fromReferenceLaserToImageCoordinates(const float dist, const float angle, const QString &reference)
{
	const QVec mapCoord = innerModel->laserTo(movableRootID, reference, dist, angle).operator*(float(bins)/float(side));
	return QVec::vec3(mapCoord(0) + 0.5*bins, 0, -mapCoord(2) + 0.5*bins);
}

inline QVec LMap::fromReferenceToImageCoordinates(const QVec &point, const QString &reference)
{
	const QVec mapCoord = innerModel->transform(movableRootID, point, reference).operator*(float(bins)/float(side));
	return QVec::vec3(mapCoord(0) + 0.5*bins, 0, -mapCoord(2) + 0.5*bins);
}

inline void LMap::addToLaserCoordinates(const int x, const int z)
{
	if (x >= 0 and x < bins and z >= 0 and z < bins)
	{
		const int64_t t = int64_t(mapLaser.at<uchar>(z, x)) + 100;
		mapLaser.at<uchar>(z, x) = t>255 ? 255 : t;
	}
};
inline void LMap::addToRGBDsCoordinates(const int x, const int z)
{
	if (x >= 0 and x < bins and z >= 0 and z < bins)
	{
		const int64_t t = int64_t(mapRGBDs.at<uchar>(z, x)) + 100;
		mapRGBDs.at<uchar>(z, x) = t>255 ? 255 : t;
	}
};


