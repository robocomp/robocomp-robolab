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
#include "specificworker.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

// setup_robot_links removed. URDFMeshLoader is used instead.

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

    m_device = rtcNewDevice(NULL);
    m_scene = rtcNewScene(m_device);
    rtcSetSceneFlags(m_scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcSetSceneBuildQuality(m_scene, RTC_BUILD_QUALITY_LOW);
    
    m_mesh_loader = new URDFMeshLoader(m_device, m_scene);

	std::string robot_name = configLoader.get<std::string>("Robot.name");
	if(robot_name == "Shadow")
	{
		robot = Robots::Shadow;
        m_mesh_loader->loadSingleSTL("robots/Shadow/shadow.stl");
	}
	else if(robot_name == "P3Bot")
	{
		robot = Robots::P3Bot;
        m_mesh_loader->loadURDF("robots/P3Bot/P3Bot.urdf", "robots/P3Bot/");
	}
	else
	{
		std::cerr << "Unknown robot name: " << robot_name << std::endl;
		exit(1);
	}

	floor_z = configLoader.get<double>("Floor.z");
	top_z = configLoader.get<double>("Top.z");
	dilate = configLoader.get<double>("Dilate");
    
    rtcCommitScene(m_scene);

	RTCBounds bounds;
	rtcGetSceneBounds(m_scene, &bounds);
	printf("Embree Scene Bounds: Min(%f, %f, %f) Max(%f, %f, %f)\n", 
		bounds.lower_x, bounds.lower_y, bounds.lower_z,
		bounds.upper_x, bounds.upper_y, bounds.upper_z);
}


void SpecificWorker::compute()
{
;
}


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}


RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
	RoboCompLidar3D::TColorCloudData ret{};
	//implementCODE
	if(robot == Robots::Shadow)
	{
		
	}
	else if(robot == Robots::P3Bot)
	{
		
	}

	return ret;
}

inline bool check_if_valid(RTCScene scene, const RoboCompLidar3D::TPoint& point, float dilate=0.05){
    // Scale point mm -> meters
    float px = point.x * 0.001f;
    float py = point.y * 0.001f;
    float pz = point.z * 0.001f;

    // Use 6 orthogonal rays to approximate a 1cm radius query. Max error is diagonals = ~1.4cm
    const float dirs[6][3] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };

    for(int i=0; i<6; ++i) {
        RTCRayHit rayhit;
        rayhit.ray.org_x = px;
        rayhit.ray.org_y = py;
        rayhit.ray.org_z = pz;
        rayhit.ray.dir_x = dirs[i][0];
        rayhit.ray.dir_y = dirs[i][1];
        rayhit.ray.dir_z = dirs[i][2];
        rayhit.ray.tnear = 0.0f;
        rayhit.ray.tfar = dilate;
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        
        rtcIntersect1(scene, &rayhit, nullptr);
        
        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            return false; // Invalid because it's within 1cm of the robot
        }
    }
    
    return true; // Valid point
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    // 1. Iniciar peticiones asíncronas de inmediato para ganar tiempo de red
    auto helios_future = lidar3d_proxy->getLidarDataAsync(name, start, len, decimationDegreeFactor);
    
    std::future<RoboCompLidar3D::TData> bpearl_future;
    const bool use_bpearl = (robot == Robots::Shadow);
    if (use_bpearl)
        bpearl_future = lidar3d1_proxy->getLidarDataAsync(name, start, len, decimationDegreeFactor);

    // 2. Mientras la red trabaja, actualizamos la cinemática del robot
    if(robot == Robots::P3Bot) {
        try {
            auto j_state = kinovaarm_proxy->getJointsState();
            std::map<int, float> joint_map;
            for(size_t i = 0; i < j_state.joints.size(); ++i) {
                joint_map[j_state.joints[i].id] = j_state.joints[i].angle;
                joint_map[i+1] = j_state.joints[i].angle;
            }
            m_mesh_loader->updateJoints(joint_map); // Esto debe llamar internamente a rtcCommitScene
        } catch(const Ice::Exception& e) {
            std::cerr << "SpecificWorker: Could not get joint state: " << e << std::endl;
        }
    }

    // 3. Recoger datos (helios es el buffer base)
    RoboCompLidar3D::TData buffer = helios_future.get();

    if (use_bpearl) {
        RoboCompLidar3D::TData bpearl_data = bpearl_future.get();
        buffer.points.insert(
            buffer.points.end(),
            std::make_move_iterator(bpearl_data.points.begin()),
            std::make_move_iterator(bpearl_data.points.end())
        );
        buffer.timestamp = std::min(buffer.timestamp, bpearl_data.timestamp);
    }

    // 4. Filtrado ULTRA-OPTIMIZADO In-Place
    if (name != "all") {
        const bool want_invalid = (name == "invalid");
        
        // Usamos remove_if para evitar crear nuevos vectores. 
        // Esto mueve los elementos válidos al principio del vector original.
        auto it_end = std::remove_if(buffer.points.begin(), buffer.points.end(), 
            [&](const RoboCompLidar3D::TPoint& p) {
                // Filtro rápido de Z (Shortcut para no llamar a Embree si no hace falta)
                bool logic_valid = (p.z > floor_z && p.z < top_z);
                
                // Si pasa el filtro Z, chequeamos colisión con la malla del robot
                if (logic_valid) {
                    logic_valid = check_if_valid(m_scene, p, dilate); // Aquí vive el rtcPointQuery (1cm margin)
                }

                // Si queremos los "invalid", invertimos la lógica
                bool final_res = want_invalid ? !logic_valid : logic_valid;
                
                // remove_if elimina si la condición es TRUE, por eso negamos el resultado final
                return !final_res;
            });

        // Ajustamos el tamaño del vector sin liberar la memoria ya reservada (capacidad)
        buffer.points.erase(it_end, buffer.points.end());
    }

    std::cout << "Num Points After Filter: " << buffer.points.size() << std::endl;
    return buffer;
}



RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
	RoboCompLidar3D::TDataImage ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
	RoboCompLidar3D::TDataCategory ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	RoboCompLidar3D::TData ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
	//implementCODE

	return ret;
}



/**************************************/
// From the RoboCompKinovaArm you can call this methods:
// RoboCompKinovaArm::bool this->kinovaarm_proxy->closeGripper()
// RoboCompKinovaArm::TPose this->kinovaarm_proxy->getCenterOfTool(ArmJoints referencedTo)
// RoboCompKinovaArm::TGripper this->kinovaarm_proxy->getGripperState()
// RoboCompKinovaArm::TJoints this->kinovaarm_proxy->getJointsState()
// RoboCompKinovaArm::TToolInfo this->kinovaarm_proxy->getToolInfo()
// RoboCompKinovaArm::void this->kinovaarm_proxy->moveJointsWithAngle(TJointAngles angles)
// RoboCompKinovaArm::void this->kinovaarm_proxy->moveJointsWithSpeed(TJointSpeeds speeds)
// RoboCompKinovaArm::void this->kinovaarm_proxy->openGripper()
// RoboCompKinovaArm::void this->kinovaarm_proxy->setCenterOfTool(TPose pose, ArmJoints referencedTo)
// RoboCompKinovaArm::bool this->kinovaarm_proxy->setGripperPos(float pos)

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompKinovaArm you can call this methods:
// RoboCompKinovaArm::bool this->kinovaarm1_proxy->closeGripper()
// RoboCompKinovaArm::TPose this->kinovaarm1_proxy->getCenterOfTool(ArmJoints referencedTo)
// RoboCompKinovaArm::TGripper this->kinovaarm1_proxy->getGripperState()
// RoboCompKinovaArm::TJoints this->kinovaarm1_proxy->getJointsState()
// RoboCompKinovaArm::TToolInfo this->kinovaarm1_proxy->getToolInfo()
// RoboCompKinovaArm::void this->kinovaarm1_proxy->moveJointsWithAngle(TJointAngles angles)
// RoboCompKinovaArm::void this->kinovaarm1_proxy->moveJointsWithSpeed(TJointSpeeds speeds)
// RoboCompKinovaArm::void this->kinovaarm1_proxy->openGripper()
// RoboCompKinovaArm::void this->kinovaarm1_proxy->setCenterOfTool(TPose pose, ArmJoints referencedTo)
// RoboCompKinovaArm::bool this->kinovaarm1_proxy->setGripperPos(float pos)

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d1_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d1_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d1_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

