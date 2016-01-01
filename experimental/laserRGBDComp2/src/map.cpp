#include "map.h"

LMap::LMap(float side_, int32_t bins_, float laserRange_)
{
	Q_ASSERT(0.5*side_ > 1.25 * laserRange);
	side = side_;
	bins = bins_;
	laserRange = laserRange_;
	lastForget = QTime::currentTime();

	map = cv::Mat(bins, bins, CV_8UC1, cv::Scalar(0));
	cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
}

cv::Mat offsetImageWithPadding(const cv::Mat& originalImage, int offsetX, int offsetY, cv::Scalar backgroundColour)
{
	auto padded = cv::Mat(originalImage.rows + 2 * abs(offsetY), originalImage.cols + 2 * abs(offsetX), CV_8UC1, backgroundColour);
	originalImage.copyTo(padded(cv::Rect(abs(offsetX), abs(offsetY), originalImage.cols, originalImage.rows)));
	return cv::Mat(padded, cv::Rect(abs(offsetX) + offsetX, abs(offsetY) + offsetY, originalImage.cols, originalImage.rows));
}

void LMap::update(RoboCompLaser::TLaserData *laserData, InnerModel *innerModel, QString movableRootID, QString robotID, QString laserID)
{
	// First, if the robot is far from the movable root, move such reference.
	const QVec relPose = innerModel->transform(movableRootID, robotID);
// 	innerModel->transform("root", movableRootID).print("robot");
// 	relPose.print("pose");
	float distFromReference = innerModel->transform(movableRootID, robotID).norm2();
	float margin = 0.5*side-laserRange;
	if (distFromReference >= margin)
	{
// 		relPose.print("\nmove rel pose");
		QVec relPosePixels = relPose;
// 		relPosePixels.print("eo1");
		printf("%f %f ---- %f\n", float(bins), float(side), float(bins)/float(side));
		relPosePixels = relPosePixels.operator*(float(bins)/float(side));
// 		relPosePixels.print("eo2");
		map = offsetImageWithPadding(map, relPosePixels(0), -relPosePixels(2), cv::Scalar(0));
		QString mrID_parent = innerModel->getParentIdentifier(movableRootID);
		qDebug() << mrID_parent;
		QVec pFromMRIDp = innerModel->transform(mrID_parent, robotID);
		innerModel->transform6D("root", movableRootID).print("MR1");
		innerModel->updateTransformValues(movableRootID, pFromMRIDp(0), pFromMRIDp(1), pFromMRIDp(2), 0,0,0);
		innerModel->transform6D("root", movableRootID).print("MR2");
		innerModel->transform(movableRootID, robotID).print("rel pose after");
	}

	// Decrease the obstacle certainty
	const float forgetRate = 20;
	const int forgetSubtract = float(lastForget.elapsed()*forgetRate)/1000.;
	if (forgetSubtract > 0)
	{
		lastForget = QTime::currentTime();
		for (int x=0; x<bins; x++)
		{
			for (int z=0; z<bins; z++)
			{
				const int64_t t = map.at<uchar>(z, x);
				map.at<uchar>(z, x) = t>=forgetSubtract ? t-forgetSubtract : 0;
			}
		}
	}

	
	for (auto datum : *laserData)
	{
		if (datum.dist < laserRange)
		{
			const QVec mapCoord = innerModel->laserTo(movableRootID, laserID, datum.dist, datum.angle).operator*(float(bins)/float(side));
			int xImageCoord =  mapCoord(0) + 0.5*bins;
			int zImageCoord = -mapCoord(2) + 0.5*bins;
			
			auto addToCoordinates = [](const int x, const int z, cv::Mat &map, const int bins)
			{
				if (x >= 0 and x < bins and z >= 0 and z < bins)
				{
					const int64_t t = map.at<uchar>(z, x) + 25;
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

	
	cv::imshow("map", map);
	cv::waitKey(1);
}

RoboCompLaser::TLaserData LMap::getData()
{
	RoboCompLaser::TLaserData ret;
	return ret;
}


