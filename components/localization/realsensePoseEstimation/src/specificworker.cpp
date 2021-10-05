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
        cameras_dict[key].pipe.stop();
	}

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	num_cameras = std::stoi(params["num_cameras"].value);
	print_output = (params["print"].value == "true") or (params["print"].value == "True");
    float rx, ry, rz, tx, ty, tz;
    std::string name;

	ifstream f_ent;

	rx = std::stof(params["origen_rx"].value);
	ry = std::stof(params["origen_ry"].value);
	rz = std::stof(params["origen_rz"].value);
	tx = std::stof(params["origen_tx"].value);
	ty = std::stof(params["origen_ty"].value);
	tz = std::stof(params["origen_tz"].value);

	FullPoseEstimation_setInitialPose(tx,ty,tz,rx,ry,rz);

	///Leemos todas las camaras del config
    for(int i=0; i<num_cameras; i++)
    {        
        PARAMS param_camera;
        name = params["name_"+std::to_string(i)].value;
        std::cout<<"Cargando: "<<name<<std::endl;
        param_camera.device_serial = params["device_serial_"+std::to_string(i)].value;
        rx = std::stof(params["rx_"+std::to_string(i)].value);
        ry = std::stof(params["ry_"+std::to_string(i)].value);
        rz = std::stof(params["rz_"+std::to_string(i)].value);
        tx = std::stof(params["tx_"+std::to_string(i)].value);
        ty = std::stof(params["ty_"+std::to_string(i)].value);
        tz = std::stof(params["tz_"+std::to_string(i)].value);
        has_odometry =  params.at("odometry").value == "true" or (params.at("odometry").value == "True");;
        ///Ejes de de camara respecto a robot
        param_camera.robot_camera = Eigen::Translation3f(Eigen::Vector3f(tx/1000,ty/1000,tz/1000));
        param_camera.robot_camera.rotate(Eigen::AngleAxisf (rx,Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
        ///Ejes de la camara respecto a origen
        param_camera.origen_camera.matrix() = param_camera.robot_camera.matrix() * this->origen_robot.matrix();

        try
        {
            rs2::config cfg;
            cfg.enable_device(param_camera.device_serial);
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

            ///CARGA DEL JSON
            rs2::pipeline_profile profile = cfg.resolve( param_camera.pipe);
            rs2::device dev = profile.get_device();
            rs2::tm2 tm2(dev);

            if (tm2)
            {
                rs2::pose_sensor pose_sensor = dev.first<rs2::pose_sensor>() ;
                param_camera.odometer = new rs2::wheel_odometer(pose_sensor);
                std::string etc_path = name;

                //replace(etc_path.begin(),etc_path.end(),'/', ' ');
                ///Leemos el .json
                f_ent.open("etc/" + etc_path +".json");

                if (f_ent.is_open()) {
                    std::cout<<etc_path<<".json"<<" se ha abierto correctamente"<<std::endl;
                    std::vector<unsigned char> cadena;
                    char aux = ' ';
                    while (f_ent.get(aux)) {
                        cadena.push_back(aux);
                        //std::cout<<aux;
                    }

                    f_ent.close(); ///Cerramos el .json

                    ///Cargamos el .json en la odometria
                    param_camera.odometer->load_wheel_odometery_config(cadena);
                }
                    else
                        std::cout<<"No se pudo abrir el "<<etc_path<<".json"<<std::endl;
            }
            /// Start pipeline with chosen configuration
            param_camera.pipe.start(cfg);

        }catch(const std::exception& e)
        {
            std::cout << e.what() << std::endl;
            qFatal("Unable to open device, please check config file");
        }

        ///Incluimos la camara generada en el diccionario de camaras
        cameras_dict.emplace(name,param_camera);
        std::cout<<"Camara añadida: "<<name<<std::endl;
    }
	return true;
}

///workaround => using serial value not working on actual api version
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
    RoboCompGenericBase::TBaseState Base;
    bool odometry_flag = false;
    if (has_odometry){
        try
        {
            this->differentialrobot_proxy->getBaseState(Base);
            odometry_flag = true;
        }
        catch(const std::exception& e)
        {
            //std::cerr << e.what() << '\n';
            qInfo() << "¡¡¡¡I dont have odometry!!!!";
        }
    }
    
    
    
    std::string text;
    for (const auto &[key, value] : cameras_dict) {
        if (odometry_flag){
        rs2_vector v;
        v.z = -Base.advVz/1000;
        v.y = 0;
        v.x = 0;
        qInfo () <<Base.advVx<< Base.advVz;
        cameras_dict[key].odometer->send_wheel_odometry(0,0,v);
        }
        
        
        auto frames = value.pipe.wait_for_frames();
        /// Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        /// Cast the frame to pose_frame and get its data, params->(https://dev.intelrealsense.com/docs/rs-pose)
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        
            Eigen::Vector3f vecQuat(pose_data.rotation.x, -pose_data.rotation.z, pose_data.rotation.y);        
        Eigen::Affine3f camera_world(Eigen::Translation3f(Eigen::Vector3f(pose_data.translation.x,-1.0 * pose_data.translation.z,pose_data.translation.y)));
        
        ///Realizamos trasformación
        bufferMutex.lock();

        ///Rotacion respecto al arranque
        vecQuat = cameras_dict[key].origen_camera.linear()*vecQuat;
        Eigen::Quaternion<float> quatCam(pose_data.rotation.w, vecQuat.x(), vecQuat.y(), vecQuat.z());
        cameras_dict[key].quatCam = quatCam;

        ///Traslacion respecto al arranque
        cameras_dict[key].translation = cameras_dict[key].origen_camera.linear() * camera_world.translation() + this -> origen_robot.translation();
        /*
        cameras_dict[key].origen_world.linear() = camera_world.linear();
        cameras_dict[key].origen_world.translation() = cameras_dict[key].origen_camera.linear() * camera_world.translation() + this -> origen_robot.translation();
        cameras_dict[key].translation.x() = cameras_dict[key].origen_world.matrix().coeff(0,3) ;
        cameras_dict[key].translation.y() = cameras_dict[key].origen_world.matrix().coeff(1,3) ;
        cameras_dict[key].translation.z() = cameras_dict[key].origen_world.matrix().coeff(2,3) ;
        */
        cameras_dict[key].mapper_confidence = pose_data.mapper_confidence;
        cameras_dict[key].tracker_confidence = pose_data.tracker_confidence;

        bufferMutex.unlock();
    }
     ///Muestra por consola si se ha solicitado en el config
        if(print_output)
        {
            FullPoseEstimation_getFullPoseEuler();
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

	RoboCompFullPoseEstimation::FullPoseEuler ret;
	ret.source = "realsense";

	std::map<string, PARAMS>::iterator it=cameras_dict.begin();
	float sigmaAcu;
	///Inicializamos el acumulador con el primero que no tenga confianza 0
	do{
	    sigmaAcu = it->second.tracker_confidence;
	    if (sigmaAcu == 0){
	        it++;
	    }
	}while (sigmaAcu == 0 && it != cameras_dict.end());
 
	Eigen::Vector3f trasAcu = it->second.translation;        //Traslación
    Eigen::Quaternion<float> quatAcu = it->second.quatCam; //Rotación
	it++;

	///Valores para normalizar sigma
	float const sigmaDifMin = -3.0;
	float const sigmaDifMax = 3.0;
	float sigmaNormCam = 0.0;
	float sigmaNormAcu = 0.0;

	///Leemos todas las camaras
	for ( ; it!=cameras_dict.end(); ++it) {

	    float sigmaCam = it->second.tracker_confidence;

	    if (sigmaCam != 0.0)
        {
            ////Normalizar la dif entre sigmaAcu y sigmaOut
            float sigmaDif = sigmaCam - sigmaAcu;
            ///Noramalización de [-3,3] a [0,1]
            sigmaNormCam = (sigmaDif - sigmaDifMin) / (sigmaDifMax - sigmaDifMin); ///esta es para el dos
            sigmaNormAcu = 1.0 - sigmaNormCam; ///esta para el uno

            std::cout<< " Confianza cámara 1: " << sigmaAcu << std::endl;
            std::cout<< " Confianza cámara 2: " << sigmaCam << std::endl;
            std::cout<< " Diferencia entre confianzas (cámara 2 - cámara 1): " << sigmaDif << std::endl;

            ///Traslación
            Eigen::Vector3f trasCam = it->second.translation;
            std::cout << "Cam_acu" << std::endl << trasAcu << std::endl << it->first << std::endl << trasCam << std::endl;
            ///Media de traslaciones respecto a sigma
            trasAcu = (trasCam * sigmaNormCam) + (trasAcu * sigmaNormAcu);

            ///Rotación

            Eigen::Quaternion<float> quatCam = it->second.quatCam;
            quatAcu=quatAcu.slerp(sigmaNormAcu,quatCam);

            
            sigmaAcu=(sigmaAcu + sigmaCam)/2;///Necesitamos otra idea de media de sigma
        }
    }

    //Eigen::Vector3f ang = quatAcu.matrix().eulerAngles(0,1,2);
    Eigen::Vector3f ang = quaternion_to_euler_angle(quatAcu.w(), quatAcu.x(), quatAcu.y(), quatAcu.z());
    ///Retornos
    ret.x = trasAcu.x()*1000;
    ret.y = trasAcu.y()*1000;
    ret.z = trasAcu.z()*1000;
    ret.rx = addAng180(ang.x(),initAngles.x());
    ret.ry = addAng180(ang.y(),initAngles.y());
    ret.rz = addAng180(ang.z(),initAngles.z());
    ret.confidence = sigmaAcu;

    if(print_output)
    {
        std::cout<<std::setprecision(3)
            << std::fixed<< "Resultado: " <<
        " Sigma normalizado cámara 1: " << sigmaNormAcu <<
        " Sigma normalizado cámara 2: " << sigmaNormCam <<
        " Traslación media (x,y,z): " <<
        ret.x << ", " <<
        ret.y << ", " <<
        ret.z <<
        " Grados media (x, y, z): " <<
        180*ret.rx/M_PI << ", " <<
        180*ret.ry/M_PI << ", " <<
        180*ret.rz/M_PI <<
        std::endl;
        }
	    
	return ret;
}

RoboCompFullPoseEstimation::FullPoseMatrix SpecificWorker::FullPoseEstimation_getFullPoseMatrix()
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    RoboCompFullPoseEstimation::FullPoseMatrix fullMatrix;
    fullMatrix.source = "camera_side";
    /*
    fullMatrix.m00 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(0,0);
    fullMatrix.m01 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(0,1);
    fullMatrix.m02 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(0,2);
    fullMatrix.m03 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(0,3);
    fullMatrix.m10 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(1,0);
    fullMatrix.m11 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(1,1);
    fullMatrix.m12 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(1,2);
    fullMatrix.m13 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(1,3);
    fullMatrix.m20 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(2,0);
    fullMatrix.m21 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(2,1);
    fullMatrix.m22 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(2,2);
    fullMatrix.m23 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(2,3);
    fullMatrix.m30 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(3,0);
    fullMatrix.m31 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(3,1);
    fullMatrix.m32 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(3,2);
    fullMatrix.m33 = cameras_dict[fullMatrix.source].origen_camera.matrix().coeff(3,3);*/
    return fullMatrix;
}
void SpecificWorker::FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz)
{
    std::lock_guard<std::mutex> lock(bufferMutex);
    ///Ejes del robot despecto al origen-mapa
    initAngles.x() = rx;
	initAngles.y() = ry;
	initAngles.z() = rz;

    ///Matriz de origen
    this->origen_robot=Eigen::Translation3f(Eigen::Vector3f(x/1000,y/1000,z/1000));
    this->origen_robot.rotate(Eigen::AngleAxisf (initAngles.x(), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (initAngles.y(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(initAngles.z(), Eigen::Vector3f::UnitZ()));
    std::cout<<"New origen:"<<std::endl<<this->origen_robot.matrix()<<endl;
    for (const auto &[key, value] : cameras_dict) {
        cameras_dict[key].origen_camera = cameras_dict[key].robot_camera.matrix() * this->origen_robot.matrix();
        
    }
}

Eigen::Vector3f SpecificWorker::quaternion_to_euler_angle(float qw, float qx, float qy, float qz){
    float auxZ;
    float auxY;
    float auxX;

    //auxZ = atan2(2*qy*qw-2*qx*qz, 1- 2*qy*qy - 2*qz*qz);
    //auxX=0;
    //auxY=0;
    auxX = atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
    auxY = asin(2*(qw*qy-qz*qx));
    auxZ = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy + qz*qz));
    
    //auxY = asin(2*qx*qy + 2*qz*qw);
    //auxX = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz);


    //float equal = qx*qy + qz*qw;
    //if (equal < 0.5+pow(10,-5) and equal > 0.5-pow(10,-5)) {
    //    auxZ = 2.0 * atan2(qx, qw);
    //    auxX = 0.0;
    //}
    //if (equal < -0.5+pow(10,-5) and equal > -0.5-pow(10,-5)) {
    //    auxZ = -2.0 * atan2(qx, qw);
    //    auxX = 0.0;
    //}

    Eigen::Vector3f ret;
    ret.x() = auxX;
    ret.y() = auxY;
    ret.z() = auxZ;
    return ret;
}
float SpecificWorker::addAng180(float ang, float angAdd){
    float angOut = ang + angAdd;
    if (angOut<-M_PI)
        return (angOut + (2*M_PI));
    else {
        if(angOut>M_PI)
            return (angOut - (2*M_PI));
        else
            return angOut;
            }
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

