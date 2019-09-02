#if COMPILE_PLAYER==1

#include "playerhandler.h"
/**
 * \brief Default constructor
 * @param TMechParams Some mechanical params read form config file
*/
PlayerHandler::PlayerHandler(RoboCompDifferentialRobot::TMechParams _params): mechParams(_params) 
{
  qDebug()<<"Starting ..";
	QString s=mechParams.device.c_str();	
	/// Connect to the player server
	client = new PlayerClient(s.split(":")[0].toStdString(),s.split(":")[1].toUInt());
	/// Open the Position interface
 	pos= new Position2dProxy (client,0);	
	pos->SetMotorEnable(true);
    mutex = new QMutex();
	
    
// 	client = new gazebo::Client();
// 	client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
	/// Open the Simulation Interface
// 	simIface = new gazebo::SimulationIface();
// 	simIface->Open(client, "default");
	/// Open the Position interface
// 	posIface = new gazebo::PositionIface();
// 	posIface->Open(client, mechParams.device.c_str());
// 	
// 	posIface->data->cmdEnableMotors = 1;
// 
	//rotation matrix
	r00=1.f;r01=0.f;r10=0.f;r11=1.f;
	t0=0.f;t1=0.f;
	alfa=0.f;	
	adv = 0.f;
	rot = 0.f;
	sendCommand = false;
	qDebug()<<"Starting PlayerHandler...";
// 	readMechParams();
}
/**
 * \brief Destructor
 */
PlayerHandler::~PlayerHandler()
{
 client->Stop();
 delete mutex;
}

void PlayerHandler::compute()
{
	computeBaseState();
	mutex->lock();
	if(sendCommand)
	{
		result = executeSpeed(adv,rot);
		sendCommand = false;
	}
	mutex->unlock();
// 	posIface->Unlock();
}
// 
// 
bool PlayerHandler::executeSpeed(float adv,float rot)
{

// 	posIface->data->cmdVelocity.pos.x = adv/1000.;
// 	posIface->data->cmdVelocity.yaw = rot;
    pos->SetSpeed(adv/1000,-rot);
	rDebug("execute speed ");
	return true;
}
// 
// /**
//  * Initialization variables and connects
//  * @param m QMutex for bState data
//  * @param mesDim_ deprecated
//  */
// void GazeboHandler::initMutex(QMutex *m)
// {
// 	
// }
/**
 * Set the base speed in rads/sg
 *
 * @param adv New base speed en mm/sg
 * @param rot New base rot speed en rads/sg
 * @return true if command executed successfully
 */
bool PlayerHandler::setSpeedBase(float _adv,float _rot){
	mutex->lock();
		adv = _adv;
		rot = _rot;
		sendCommand = true;
	mutex->unlock();
	QTime t=QTime::currentTime();
	while(true)
	{
		if(t.elapsed() > 100)
			return false;
		mutex->lock();
		if(sendCommand == false){
		  	mutex->unlock();
			return result;
		}
		mutex->unlock();
		usleep(10);
	}
	return false;
}
/**
 * Stops base
 * @return true if sucess
 */
bool PlayerHandler::stopBase()
{
	return setSpeedBase(0.f,0.f);
}
// 
void PlayerHandler::computeBaseState()
{
	static QTime reloj=QTime::currentTime();
	static float xant;
	static float yant;
// 	float x = posIface->data->pose.pos.y*1000.;
// 	float z = posIface->data->pose.pos.x*1000.;
// 	client->Read();
	float x = pos->GetYPos()*1000.;
	float z = pos->GetXPos()*1000.;
	float dl = float ( x - xant );
	float dr = float ( z - yant );
	xant = bState.x;
	yant = bState.z;
	float incA = sqrt(dl*dl+dr*dr);
	mutex->lock();
		//Compute increments through robot kinematics
		client->Read();
		bState.adv = incA;
// 		bState.rotV = -posIface->data->velocity.yaw;
		bState.rotV = -pos->GetYawSpeed();
		bState.advV = incA / (float) reloj.restart() * 1000;
		bState.rotV = -pos->GetYawSpeed() / (float) reloj.restart() * 1000;
		bState.isMoving = (bState.advV!=0 or bState.rotV!=0);
		//position
// 		bState.alpha = posIface->data->pose.yaw - alfa;
		bState.alpha = pos->GetYaw();
		bState.x = ( r00*x + r01*z ) - (r00*t0 + r01*t1);
		bState.z = ( r10*x + r11*z ) - (r10*t0 + r11*t1);
// 		qDebug()<<bState.x<<bState.z<<rtod(bState.alpha);
	mutex->unlock();
}

/**
 * Return current base state
 * @return bState
 */
RoboCompDifferentialRobot::TBaseState PlayerHandler::getBaseState(){
	return bState;
}
/**
 * Return current base mechanical params
 * @return TMechParams
 */
RoboCompDifferentialRobot::TMechParams PlayerHandler::getMechParams()
{
	return mechParams;
}
// 
/**
 * Reset odometer
 * @return true if command was sended successfully, else return false
 */
bool PlayerHandler::resetOdometer(){
	if(setSpeedBase(0.f,0.f))
	{
	      mutex->lock();
			client->Read();
// 			alfa = posIface->data->pose.yaw;
			alfa = pos->GetYaw();
			r00 = sin(alfa);
			r01 = -cos(alfa);
			r10 = cos(alfa);
			r11 = sin(alfa);
// 			t0 = posIface->data->pose.pos.y*1000.;
// 			t1 = posIface->data->pose.pos.x*1000.;	     
			t0 = pos->GetYPos()*1000.;
			t1 = pos->GetXPos()*1000.;	     
	      mutex->unlock();
	      return true;
	}
	else
	      return false;
}
/**
* Set odometer to a specified state
* @param _bState State to set odometer values
* @return true if command was sended successfully, else return false
*/
bool PlayerHandler::setOdometer(RoboCompDifferentialRobot::TBaseState _bState){
	if(setSpeedBase(0.f,0.f))
	{
		mutex->lock();
// 			posIface->data->pose.pos.x=_bState.x;
// 			posIface->data->pose.pos.y=_bState.z;
// 			posIface->data->pose.yaw=_bState.alpha;
// 			posIface->data->velocity.pos.x=0.;
// 			posIface->data->velocity.yaw=0.;    			
			pos->SetOdometry(_bState.x,_bState.z,_bState.alpha);
		mutex->unlock();
		return true;
	}
	else
	      return false;
}

void PlayerHandler::readMechParams()
{
      //Read from gazebo xml model
      mechParams.wheelRadius = 10;
      mechParams.axisLength = 15;
      mechParams.encoderSteps = 10000;

}

#endif
