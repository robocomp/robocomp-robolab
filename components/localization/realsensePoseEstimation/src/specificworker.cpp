/*
 *    Copyright (C)2019 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check): GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	for (const auto &[key, value] : cameras_dict) {
	    delete(cameras_dict[key].odometer);
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	num_cameras = std::stoi(params["num_cameras"].value);
	print_output = (params["print"].value == "true") or (params["print"].value == "True");
    float rx, ry, rz, tx, ty, tz;
    std::string name;

	ifstream f_ent;

	rx = (PI * (std::stof(params["origen_rx"].value))) / 180;
	ry = (PI * (std::stof(params["origen_ry"].value))) / 180;
	rz = (PI * (std::stof(params["origen_rz"].value)))/180;
	tx = std::stof(params["origen_tx"].value);
	ty = std::stof(params["origen_ty"].value);
	tz = std::stof(params["origen_tz"].value);

	FullPoseEstimation_setInitialPose(tx,ty,tz,rx,ry,rz); //cambio recursivo de matrices falta

	//Leemos todas las camaras del config
    for(int i=0; i<num_cameras; i++)
    {
        PARAMS param_camera;
        name = params["name_"+std::to_string(i)].value;
        std::cout<<"Cargando: "<<name<<std::endl;
        param_camera.device_serial = params["device_serial_"+std::to_string(i)].value;
        param_camera.rot_init_angles.x = (PI * (std::stof(params["rx_"+ std::to_string(i)].value))) / 180;
        param_camera.rot_init_angles.y = (PI * (std::stof(params["ry_"+std::to_string(i)].value))) / 180;
        param_camera.rot_init_angles.z = (PI * (std::stof(params["rz_"+std::to_string(i)].value)))/180;
        tx = std::stof (params["tx_"+std::to_string(i)].value);
        ty = std::stof(params["ty_"+std::to_string(i)].value);
        tz = std::stof(params["tz_"+std::to_string(i)].value);

        //ejes de de camara respecto a robot
        param_camera.robot_camera = Eigen::Translation3f(Eigen::Vector3f(tx,ty,tz));
        param_camera.robot_camera.rotate(Eigen::AngleAxisf (param_camera.rot_init_angles.x,Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (param_camera.rot_init_angles.y, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(param_camera.rot_init_angles.z, Eigen::Vector3f::UnitZ()));
        // ejes de la camara respecto a origen
        param_camera.origen_camera.linear() = param_camera.robot_camera.linear() * this->origen_robot.linear();
        param_camera.origen_camera.translation() = this->origen_robot.linear() * param_camera.robot_camera.translation() + this->origen_robot.translation();
        //std::cout<<param_camera.origen_camera.matrix()<<std::endl;

        try
        {
            rs2::config cfg;
            cfg.enable_device(param_camera.device_serial);
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

            //CARGA DEL JSON
            rs2::pipeline_profile profile = cfg.resolve( param_camera.pipe);
            rs2::device dev = profile.get_device();
            rs2::tm2 tm2(dev);

            if (tm2)
            {
                rs2::pose_sensor pose_sensor = dev.first<rs2::pose_sensor>() ;
                param_camera.odometer = new rs2::wheel_odometer(pose_sensor);
                std::string etc_path = name;
                //replace(etc_path.begin(),etc_path.end(),'/', ' ');
                //Leemos el .json
                f_ent.open("etc/" + etc_path +".json");
                    if (f_ent.is_open()) {
                        std::cout<<etc_path<<".json"<<" se ha abierto correctamente"<<std::endl;
                        std::vector<unsigned char> cadena;
                        char aux = ' ';
                        while (f_ent.get(aux)) {
                            cadena.push_back(aux);
                           // std::cout<<aux;
                        }
                        //Cargamos el .json en la odometria
                        //param_camera.odometer->load_wheel_odometery_config(cadena);
                    }
                    else
                        std::cout<<"No se pudo abrir el "<<etc_path<<".json"<<std::endl;
            }
            // Start pipeline with chosen configuration
            param_camera.pipe.start(cfg);
        }catch(const std::exception& e)
        {
            std::cout << e.what() << std::endl;
            qFatal("Unable to open device, please check config file");
        }

        //Incluimos la camara generada en el diccionario de camaras
        cameras_dict.emplace(name,param_camera);
        std::cout<<"Camara añadida: "<<name<<std::endl;
    }
	return true;
}

//workaround => using serial value not working on actual api version
rs2::device SpecificWorker::get_device(const std::string& serial_number) {
    rs2::context ctx;
    while (true)
    {
        for (auto&& dev : ctx.query_devices())
            if (std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == serial_number)
                return dev;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = 20;
	timer.start(Period);
}

void SpecificWorker::compute()
{
    //RoboCompGenericBase::TBaseState Base = self.differentialrobot_proxy.getBaseState();
    ///this->genericbase_proxy->getBaseState(Base);

    euler_angle ang;
    for (const auto &[key, value] : cameras_dict) {

        rs2_vector v;
        v.z = 0; // v.z = -Base.advVz/1000;
        v.y = 0;
        v.x = 0;
        //cameras_dict[key].odometer->send_wheel_odometry(0,0,v);
        //std::cout<<"Velocidad "<<v.z<<std::endl;

        auto frames = value.pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        // Print the x, y, z values of the translation, relative to initial position

        ang = quaternion_to_euler_angle(pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z);

        Eigen::Quaternion<float> quatIn(pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z);
        Eigen::Affine3f camera_world(Eigen::Translation3f(Eigen::Vector3f(pose_data.translation.x,pose_data.translation.y,pose_data.translation.z)));
        camera_world.rotate(quatIn);

        bufferMutex.lock();
        cameras_dict[key].origen_world.linear() = camera_world.linear() * cameras_dict[key].origen_camera.linear();
        cameras_dict[key].origen_world.translation() = cameras_dict[key].origen_camera.linear() * camera_world.translation() + this->origen_robot.translation();


        //Eigen::Quaternion<float> quatOut(camera_world.linear() * this->origen_robot.linear());
        Eigen::Quaternion<float> quatOut(cameras_dict[key].origen_world.linear() ) ;
        euler_angle angOut = quaternion_to_euler_angle(quatOut.w(),quatOut.x(),quatOut.y(),quatOut.z());

        /*
        cameras_dict[key].angles.x = angOut.x;
        cameras_dict[key].angles.y = angOut.y;
        cameras_dict[key].angles.z = angOut.z;
*/


        angOut.x = angOut.x - cameras_dict[key].rot_init_angles.x;
        angOut.y = angOut.y - cameras_dict[key].rot_init_angles.y;
        angOut.z = angOut.z - cameras_dict[key].rot_init_angles.z;

        if (angOut.x<-M_PI)
            cameras_dict[key].angles.x = angOut.x + (2*M_PI);
        else {if(angOut.x>M_PI)
            cameras_dict[key].angles.x = angOut.x - (2*M_PI);
        else
            cameras_dict[key].angles.x=angOut.x;}

        if (angOut.y<-M_PI){
            cameras_dict[key].angles.y = angOut.y + (2*M_PI);
            std::cout<< "-180"<<std::endl;
        }
        else {
            if(angOut.y>M_PI){
                cameras_dict[key].angles.y = angOut.y - (2*M_PI);
            std::cout<< "180"<<std::endl;
            }
            else{
                cameras_dict[key].angles.y=angOut.y;
                std::cout<< "NAN"<<std::endl;
            }
        }

        if (angOut.z<-M_PI)
            cameras_dict[key].angles.z = angOut.z + (2*M_PI);
        else {if(angOut.z>M_PI)
            cameras_dict[key].angles.z = angOut.z - (2*M_PI);
        else
            cameras_dict[key].angles.z=angOut.z;}

        cameras_dict[key].mapper_confidence = pose_data.mapper_confidence;
        cameras_dict[key].tracker_confidence = pose_data.tracker_confidence;

        //std::cout<<"\r"<< "Quat entrada vs Quat salida: " <<std::setprecision(3)<<quatIn.w()<<" "<<quatOut.w()<<"/"<<quatIn.x()<<" "<<quatOut.x()<<"/"<<quatIn.y()<<" "<<quatOut.y()<<"/"<<quatIn.z()<<" "<<quatOut.z();
        //std::cout<<"\r" << "Ang entrada vs Ang salida: " << std::setprecision(3)<<180* ang.x/M_PI<<" "<<180*ang.x/M_PI<<" / "<<180*angAux.y/M_PI<<" "<<180* ang.y/M_PI<<" / "<<180*angAux.z/M_PI<<" "<<180*ang.z/M_PI << endl;

        bufferMutex.unlock();

        if(print_output){
            /*std::cout << "\r" << std::setprecision(3)*/
            std::cout << "Device: " << key <<std::setprecision(3)

            << std::fixed
            << cameras_dict[key].origen_world.matrix().coeff(0,3)<< " "
            << cameras_dict[key].origen_world.matrix().coeff(1,3) << " "
            << cameras_dict[key].origen_world.matrix().coeff(2,3)<< " (met) "/*
            << quatOut.x() << " "
            << quatOut.y() << " "
            << quatOut.z() << " "
            << quatOut.w() << " (quat) " << " "*/
            << 180*cameras_dict[key].angles.x/PI << " "
            << 180*cameras_dict[key].angles.y/PI << " "
            << 180*cameras_dict[key].angles.z/PI << " (grad) " << " "/*
            << cameras_dict[key].angles.x << " "
            << cameras_dict[key].angles.y << " "
            << cameras_dict[key].angles.z << "(rad)"*/
            << std::endl;

            //FullPoseEstimation_getFullPoseEuler();
        }


    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

RoboCompFullPoseEstimation::FullPoseEuler SpecificWorker::FullPoseEstimation_getFullPoseEuler()
{
	std::lock_guard<std::mutex> lock(bufferMutex);

	int sigma = 0;
	RoboCompFullPoseEstimation::FullPoseEuler ret;
	ret.source = "realsense";
	for (const auto &[key, value] : cameras_dict)
	{
	    //CALCULATE ADDITION BOTH DATA'S CAMERA
	    ret.x = ret.x + cameras_dict[key].origen_world.matrix().coeff(0,3) * cameras_dict[key].tracker_confidence;
	    ret.y = ret.y + cameras_dict[key].origen_world.matrix().coeff(1,3) * cameras_dict[key].tracker_confidence;
	    ret.z = ret.z + cameras_dict[key].origen_world.matrix().coeff(2,3) * cameras_dict[key].tracker_confidence;
	    ret.rx = ret.rx + cameras_dict[key].angles.x * cameras_dict[key].tracker_confidence;
	    ret.ry = ret.ry + cameras_dict[key].angles.y * cameras_dict[key].tracker_confidence;
	    ret.rz = ret.rz + cameras_dict[key].angles.z * cameras_dict[key].tracker_confidence;
        sigma = sigma + cameras_dict[key].tracker_confidence;

        //CALCULATE AVERAGE OF POSITION
        ret.x = ret.x / sigma;
        ret.y = ret.y / sigma;
        ret.z = ret.z / sigma;

        //CALCULATE AVERAGE OF ANGLES  (CHECK -PI to PI transition !!!!)
        ret.rx = ret.rx / sigma;
        ret.ry = ret.ry / sigma;
        ret.rz = ret.rz / sigma;

        if(print_output)
            std::cout << "\r" << "Resultado" << " " <<
                                sigma << " " <<
                                ret.x << " " <<
                                ret.y << " " <<
                                ret.z << " " <<
                                ret.rx << " " <<
                                ret.ry << " " <<
                                ret.rz << std::endl;
	}

	return ret;
}


RoboCompFullPoseEstimation::FullPoseMatrix SpecificWorker::FullPoseEstimation_getFullPoseMatrix()
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    RoboCompFullPoseEstimation::FullPoseMatrix fullMatrix;
    fullMatrix.source = "camera_side";
    std::string camera = "camera_side";
    
    fullMatrix.m00 = cameras_dict[camera].origen_camera.matrix().coeff(0,0);
    fullMatrix.m01 = cameras_dict[camera].origen_camera.matrix().coeff(0,1);
    fullMatrix.m02 = cameras_dict[camera].origen_camera.matrix().coeff(0,2);
    fullMatrix.m03 = cameras_dict[camera].origen_camera.matrix().coeff(0,3);
    fullMatrix.m10 = cameras_dict[camera].origen_camera.matrix().coeff(1,0);
    fullMatrix.m11 = cameras_dict[camera].origen_camera.matrix().coeff(1,1);
    fullMatrix.m12 = cameras_dict[camera].origen_camera.matrix().coeff(1,2);
    fullMatrix.m13 = cameras_dict[camera].origen_camera.matrix().coeff(1,3);
    fullMatrix.m20 = cameras_dict[camera].origen_camera.matrix().coeff(2,0);
    fullMatrix.m21 = cameras_dict[camera].origen_camera.matrix().coeff(2,1);
    fullMatrix.m22 = cameras_dict[camera].origen_camera.matrix().coeff(2,2);
    fullMatrix.m23 = cameras_dict[camera].origen_camera.matrix().coeff(2,3);
    fullMatrix.m30 = cameras_dict[camera].origen_camera.matrix().coeff(3,0);
    fullMatrix.m31 = cameras_dict[camera].origen_camera.matrix().coeff(3,1);
    fullMatrix.m32 = cameras_dict[camera].origen_camera.matrix().coeff(3,2);
    fullMatrix.m33 = cameras_dict[camera].origen_camera.matrix().coeff(3,3);
    return fullMatrix;
}
void SpecificWorker::FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz)
{
    //Ejes del robot despecto al origen-mapa
    this->origen_robot=Eigen::Translation3f(Eigen::Vector3f(x,y,z));
    this->origen_robot.rotate(Eigen::AngleAxisf (rx, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));

    for (const auto &[key, value] : cameras_dict) {

        cameras_dict[key].origen_camera.linear() =  cameras_dict[key].robot_camera.linear() * this->origen_robot.linear();
        cameras_dict[key].origen_camera.translation() = this->origen_robot.linear() *  cameras_dict[key].robot_camera.translation() + this->origen_robot.translation();

    }

}

SpecificWorker::euler_angle SpecificWorker::quaternion_to_euler_angle(float qw, float qx, float qy, float qz){
    float auxZ;
    float auxY;
    float auxX;

    auxZ = atan2(2*qy*qw-2*qx*qz, 1- 2*qy*qy - 2*qz*qz);
    auxY = asin(2*qx*qy + 2*qz*qw);
    auxX = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz);


    float equal = qx*qy + qz*qw;
    if (equal < 0.5+pow(10,-5) and equal > 0.5-pow(10,-5)) {
        auxZ = 2.0 * atan2(qx, qw);
        auxX = 0.0;
    }
    if (equal < -0.5+pow(10,-5) and equal > -0.5-pow(10,-5)) {
        auxZ = -2.0 * atan2(qx, qw);
        auxX = 0.0;
    }

    euler_angle ret;
    ret.x = auxX;
    ret.y = auxZ;
    ret.z = auxY;
    return ret;
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompGenericBase you can call this methods:
// this->genericbase_proxy->getBasePose(...)
// this->genericbase_proxy->getBaseState(...)

/**************************************/
// From the RoboCompGenericBase you can use this types:
// RoboCompGenericBase::TBaseState

/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// this->fullposeestimationpub_pubproxy->newFullPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

