#include "map.h"

LMap::LMap(float side_, int32_t bins_, float laserRange_)
{
	Q_ASSERT(0.5*side_ > 1.25 * laserRange);
	side = side_;
	bins = bins_;
	laserRange = laserRange_;
	lastForgetSubtract = QTime::currentTime();
	lastForgetAdd = QTime::currentTime();

	map = cv::Mat(bins, bins, CV_8UC1, cv::Scalar(0));
	cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
}


cv::Mat offsetImageWithPadding(const cv::Mat& originalImage, int offsetX, int offsetY)
{
	auto padded = cv::Mat(originalImage.rows + 2 * abs(offsetY), originalImage.cols + 2 * abs(offsetX), CV_8UC1, cv::Scalar(128));
	originalImage.copyTo(padded(cv::Rect(abs(offsetX), abs(offsetY), originalImage.cols, originalImage.rows)));
	return cv::Mat(padded, cv::Rect(abs(offsetX) + offsetX, abs(offsetY) + offsetY, originalImage.cols, originalImage.rows));
}


void LMap::update_timeAndPositionIssues(InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID)
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
		map = offsetImageWithPadding(map, relPosePixels(0), -relPosePixels(2));
		QString mrID_parent = innerModel->getParentIdentifier(movableRootID);
		QVec pFromMRIDp = innerModel->transform(mrID_parent, virtualLaserID);
		innerModel->updateTransformValues(movableRootID, pFromMRIDp(0), pFromMRIDp(1), pFromMRIDp(2), 0,0,0);
		printf("moving movableRootID %f\n", distFromReference);
	}

	// Decrease the obstacle certainty
	const float forgetRateSubtract = 12;
	const float forgetSubtract = float(forgetRateSubtract*lastForgetSubtract.elapsed())/1000.;
	if (forgetSubtract > 5)
	{
		lastForgetSubtract = QTime::currentTime();
		for (int x=0; x<bins; x++)
		{
			for (int z=0; z<bins; z++)
			{
				int64_t t = map.at<uchar>(z, x);
				if (t > 128)
				{
					t -= forgetSubtract;
					if (t<128) t = 128;
					map.at<uchar>(z, x) = t;
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
				int64_t t = map.at<uchar>(z, x);
				if (t < 128)
				{
					t += forgetAdd;
					if (t>128) t = 128;
					map.at<uchar>(z, x) = t;
				}
			}
		}
	}
}


void LMap::update_laser(RoboCompLaser::TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID)
{
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord = innerModel->laserTo(movableRootID, actualLaserID, datum.dist-(6.*side/bins), datum.angle).operator*(float(bins)/float(side));
		int xImageCoord =  mapCoord(0) + 0.5*bins;
		int zImageCoord = -mapCoord(2) + 0.5*bins;
		// Empty a line 
		const QVec mapCoordZ = innerModel->laserTo(movableRootID, actualLaserID, 0, datum.angle).operator*(float(bins)/float(side));
		int xImageCoordL =  mapCoordZ(0) + 0.5*bins;
		int zImageCoordL = -mapCoordZ(2) + 0.5*bins;
		cv::line(map, cv::Point(xImageCoord, zImageCoord), cv::Point(xImageCoordL, zImageCoordL), cv::Scalar(0), 5);
	}
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord = innerModel->laserTo(movableRootID, actualLaserID, datum.dist, datum.angle).operator*(float(bins)/float(side));
		int xImageCoord =  mapCoord(0) + 0.5*bins;
		int zImageCoord = -mapCoord(2) + 0.5*bins;
		// Draw obstacle
		if (datum.dist < laserRange)
		{
			auto addToCoordinates = [](const int x, const int z, cv::Mat &map, const int bins)
			{
				if (x >= 0 and x < bins and z >= 0 and z < bins)
				{
					const int64_t t = int64_t(map.at<uchar>(z, x)) + 100;
					map.at<uchar>(z, x) = t>255 ? 255 : t;
				}
			};
			for (auto i : std::vector<int>{-1, 0, 1})
			{
				addToCoordinates(xImageCoord+i, zImageCoord, map, bins);
				addToCoordinates(xImageCoord, zImageCoord+i, map, bins);
			}
		}
	}
}


void LMap::update_done(InnerModel *innerModel, QString movableRootID, QString virtualLaserID, QString actualLaserID, float minDist)
{
	// Empty a line 
	const QVec mapCoord = innerModel->transform(movableRootID, virtualLaserID).operator*(float(bins)/float(side));
	int xImageCoord =  mapCoord(0) + 0.5*bins;
	int zImageCoord = -mapCoord(2) + 0.5*bins;
	cv::circle(map, cv::Point(xImageCoord, zImageCoord), minDist*float(bins)/float(side), cv::Scalar(0), -1, 8, 0);

	cv::threshold(map, mapThreshold, 127, 128, cv::THRESH_BINARY);

	cv::imshow("map", map);
	cv::imshow("mapThreshold", mapThreshold);
	if (cv::waitKey(3) == 27)
		exit(0);
}


void LMap::getLaserData(RoboCompLaser::TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString virtualLaserID, int32_t laserBins, float laserFOV, float maxLength)
{
	/// Clear laser measurement
	for (int32_t i=0; i<laserBins; ++i)
	{
		(*laserData)[i].dist = maxLength;
	}

	for (int xi=0; xi<bins; xi++)
	{
		for (int zi=0; zi<bins; zi++)
		{
			if (mapThreshold.at<uchar>(zi, xi) > 0)
			{
				QVec mapCoord = QVec::vec3(-0.5*bins+xi, 0, 0.5*bins-zi).operator*(float(side)/float(bins));
				QVec mapCoordInLaser = innerModel->transform(virtualLaserID, mapCoord, movableRootID);
				float angle = atan2(mapCoordInLaser(0), mapCoordInLaser(2));
				float dist = mapCoordInLaser.norm2();
				int32_t bin = angle2bin(angle, laserFOV, laserBins);
				if ((*laserData)[bin].dist > dist)
				{
					(*laserData)[bin].dist = dist;
					mapThreshold.at<uchar>(zi, xi) = 255;
				}
			}
		}
	}
}

