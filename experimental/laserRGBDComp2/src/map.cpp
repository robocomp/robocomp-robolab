#include "map.h"

LMap::LMap(float side_, int32_t bins_, float laserRange_, InnerModel *innerModel_)
{
	Q_ASSERT(0.5*side_ > 1.25 * laserRange);
	side = side_;
	bins = bins_;
	innerModel = innerModel_;
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


void LMap::update_timeAndPositionIssues(QString movableRootID, QString virtualLaserID, QString actualLaserID)
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


void LMap::update_include_laser(RoboCompLaser::TLaserData *laserData, QString movableRootID, QString virtualLaserID, QString actualLaserID)
{
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord  = fromReferenceLaserToImageCoordinates(datum.dist-(6.*side/bins), datum.angle, actualLaserID, movableRootID);
		int xImageCoord = mapCoord(0);
		int zImageCoord = mapCoord(2);
		// Empty a line 
		const QVec mapCoordZ = fromReferenceLaserToImageCoordinates(                        0, datum.angle, actualLaserID, movableRootID);
		int xImageCoordL = mapCoordZ(0);
		int zImageCoordL = mapCoordZ(2);
		cv::line(map, cv::Point(xImageCoord, zImageCoord), cv::Point(xImageCoordL, zImageCoordL), cv::Scalar(0), 5);
	}
	for (auto datum : *laserData)
	{
		// Compute points' map coordinates
		const QVec mapCoord = fromReferenceLaserToImageCoordinates(datum.dist, datum.angle, actualLaserID, movableRootID);
		// Draw obstacle
		if (datum.dist < laserRange)
		{
			for (auto i : std::vector<int>{-1, 0, 1})
			{
				addToCoordinates(mapCoord(0)+i, mapCoord(0));
				addToCoordinates(mapCoord(0), mapCoord(2)+i);
			}
		}
	}
}

void LMap::update_include_rgbd(RoboCompRGBD::PointSeq *points, QString movableRootID, QString virtualLaserID, QString rgbdID)
{
	RTMat TRv = innerModel->getTransformationMatrix(virtualLaserID, rgbdID);
	RTMat TRr = innerModel->getTransformationMatrix("robot", rgbdID);

	uint32_t pw = 640;
	uint32_t ph = 480;
	if (points->size() == 320*240) { pw=320; ph=240; }
	if (points->size() == 160*120) { pw=160; ph=120; }
	if (points->size() == 80*60) { pw=80; ph=60; }
	for (uint32_t rr=0; rr<ph; rr+=2)
	{
		for (uint32_t cc=0; cc<pw; cc+=3)
		{
			uint32_t ioi = rr*pw+cc;
			if (ioi<points->size())
			{
				const QVec pRobot   = (TRr * QVec::vec4(points->operator[](ioi).x, points->operator[](ioi).y, points->operator[](ioi).z, 1)).fromHomogeneousCoordinates();
				//if (ioi == interest) p.print("en final");
				int minHeight = 100;
				int maxHeight = 1900;
				int minHeightNeg = -200;
				if ( (pRobot(1)>=minHeight and pRobot(1)<=maxHeight) or (pRobot(1)<minHeightNeg) )
				{
					const QVec pVirtual = (TRv * QVec::vec4(points->operator[](ioi).x, points->operator[](ioi).y, points->operator[](ioi).z, 1)).fromHomogeneousCoordinates();
					const QVec mapCoord = fromReferenceToImageCoordinates(pVirtual, virtualLaserID, movableRootID);
					for (auto i : std::vector<int>{-1, 0, 1})
					{
						addToCoordinates(mapCoord(0)+i, mapCoord(0));
						addToCoordinates(mapCoord(0), mapCoord(2)+i);
					}
				}
			}
		}
	}
}

void LMap::update_done(QString movableRootID, QString virtualLaserID, QString actualLaserID, float minDist)
{
	// Empty a line 
	const QVec mapCoord = innerModel->transform(movableRootID, virtualLaserID).operator*(float(bins)/float(side));
	int xImageCoord =  mapCoord(0) + 0.5*bins;
	int zImageCoord = -mapCoord(2) + 0.5*bins;
	cv::circle(map, cv::Point(xImageCoord, zImageCoord), minDist*float(bins)/float(side), cv::Scalar(0), -1, 8, 0);

	cv::threshold(map, mapThreshold, 127, 128, cv::THRESH_BINARY);
	  
	
	int dilation_size = 1;
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size( 2*dilation_size+1, 2*dilation_size+1), cv::Point( dilation_size, dilation_size ) );
	cv::dilate(mapThreshold, mapThreshold, element);

	cv::imshow("map", map);
	cv::imshow("mapThreshold", mapThreshold);
	if (cv::waitKey(3) == 27)
		exit(0);
}


void LMap::getLaserData(RoboCompLaser::TLaserData *laserData, QString movableRootID, QString virtualLaserID, int32_t laserBins, float laserFOV, float maxLength)
{
	/// Clear laser measurement
	for (int32_t i=0; i<laserBins; ++i)
	{
		(*laserData)[i].dist = maxLength;
	}

	const QVec lmapCoord = innerModel->laserTo(movableRootID, virtualLaserID, 0,0).operator*(float(bins)/float(side));
	int xlImageCoord =  lmapCoord(0) + 0.5*bins;
	int zlImageCoord = -lmapCoord(2) + 0.5*bins;
	int radius = (maxLength*float(bins))/float(side);
	for (int xi=0; xi<bins; xi+=2)
	{
		for (int zi=0; zi<bins; zi+=2)
		{
			if (abs(xlImageCoord-xi)<=radius and abs(zlImageCoord-zi)<=radius)
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
}

inline int32_t LMap::angle2bin(double ang, const float laserFOV, const int bins)
{
	while (ang>M_PI)  ang -= 2.*M_PI;
	while (ang<-M_PI) ang += 2.*M_PI;

	double ret;
	ang += laserFOV/2.;
	ret = (ang * bins) / laserFOV;
	return int32_t(ret);
}

inline QVec LMap::fromReferenceLaserToImageCoordinates(const float dist, const float angle, const QString &reference, const QString &movableRootID)
{
	const QVec mapCoord = innerModel->laserTo(movableRootID, reference, dist, angle).operator*(float(bins)/float(side));
	return QVec::vec3(mapCoord(0) + 0.5*bins, 0, -mapCoord(2) + 0.5*bins);
}

inline QVec LMap::fromReferenceToImageCoordinates(const QVec &point, const QString &reference, const QString &movableRootID)
{
	const QVec mapCoord = innerModel->transform(movableRootID, point, reference).operator*(float(bins)/float(side));
	return QVec::vec3(mapCoord(0) + 0.5*bins, 0, -mapCoord(2) + 0.5*bins);
}

inline void LMap::addToCoordinates(const int x, const int z)
{
	if (x >= 0 and x < bins and z >= 0 and z < bins)
	{
		const int64_t t = int64_t(map.at<uchar>(z, x)) + 100;
		map.at<uchar>(z, x) = t>255 ? 255 : t;
	}
};


