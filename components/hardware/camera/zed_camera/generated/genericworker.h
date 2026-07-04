/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include <stdint.h>
#include <grafcetStep/GRAFCETStep.h>
#include <ConfigLoader/ConfigLoader.h>
#include <QStateMachine>
#include <QEvent>
#include <QString>
#include <functional>
#include <atomic>
#include <QtCore>
#include <variant>
#include <unordered_map>
#include <fps/fps.h>


#include <CameraRGBDSimple.h>
#include <CameraRGBDSimplePub.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <IMU.h>
#include <IMUPub.h>
#include <Lidar3D.h>
#include <MediaPlaneDDS.h>

#define BASIC_PERIOD 100

using TuplePrx = std::tuple<RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr,RoboCompCameraRGBDSimplePub::CameraRGBDSimplePubPrxPtr,RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr,RoboCompIMUPub::IMUPubPrxPtr>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();

	void setPeriod(const std::string& state, int period);
	int getPeriod(const std::string& state);

	QStateMachine statemachine;
	QTimer hibernationChecker;
	std::atomic_bool hibernation = false;


	RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr camerargbdsimple_proxy;
	RoboCompCameraRGBDSimplePub::CameraRGBDSimplePubPrxPtr camerargbdsimplepub_pubproxy;
	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_pubproxy;
	RoboCompIMUPub::IMUPubPrxPtr imupub_pubproxy;

	virtual RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera) = 0;

	virtual RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData() = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor) = 0;
	virtual RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor) = 0;

	virtual std::string MediaPlaneDDS_getMediaDescriptor() = 0;


protected:
	std::unordered_map<std::string, std::unique_ptr<GRAFCETStep>> states;
	ConfigLoader configLoader;
	FPSCounter fps;




private:

public slots:
	virtual void initialize() = 0;
	virtual void compute() = 0;
	virtual void emergency() = 0;
	virtual void restore() = 0;
	void hibernationCheck();
	void hibernationTick();
	
signals:
	void kill();
	void goToEmergency();
	void goToRestore();
};

#endif
