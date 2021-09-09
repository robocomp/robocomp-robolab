/***********************************************************************/
/**                                                                    */
/** giraff_manager.h                                                   */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************///


#ifndef _GIRAFF_MANAGER_H_
#define _GIRAFF_MANAGER_H_


#define PI 3.14159265
#define DISTANCE_B_WHEELS 0.467              //Dinstance (m) between both wheels
#define MAX_STALK_HEIGHT 1.410
#define MIN_STALK_HEIGHT 1.160

//********
// FLAGS
//********
// Activate this to show some debug information
//#define _GIRAFF_MANAGER_DEBUG_
// Activate if you want to filter the "set tilt_angle_from_home" command from the Pilot
#define _FILTER_TILT_FROM_PILOT_
// Activate if you want to monitor the actual parameters of the AVR, instead of the sent parameters
//#define _MONITOR_ACTUAL_AVR_PARAMETERS_

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "giraff_serial.h"




//---------------------------------- HEADERS ------------------------------------//

/*********************************************************************************/
/**                                                                              */
/** class GiraffManagerException                                                 */
/**                                                                              */
/** An exception to throw in case of fatal error                                 */
/**                                                                              */
/*********************************************************************************/
class GiraffManagerException
{
    public:
    GiraffManagerException(const std::string& message) : message(message) {}
    ~GiraffManagerException() throw () {}
    virtual const char* what() const throw() {return message.c_str();}

    private:
    std::string message;
};


/*********************************************************************************/
/**                                                                              */
/** class StopAction                                                             */
/**                                                                              */
/** This implements an action to stop the Giraff. It is used when the            */
/** Giraff PC activates its DTR flag to begin communication, we stop the Giraff  */
/** for security and send the welcome message                                    */
/**                                                                              */
/*********************************************************************************/
class StopGiraffAction : public Action
{
    public:
    StopGiraffAction(GiraffAVR& giraffAVR) : giraffAVR(giraffAVR) {}
    virtual ~StopGiraffAction() {}
    virtual bool doAction() {return giraffAVR.setInt32("mode",0) &&
                                    giraffAVR.setFloat("a",0) &&
                                    giraffAVR.setFloat("v",0) &&
                                    giraffAVR.setFloat("vg",0) &&
                                    giraffAVR.setFloat("p",0); }

    private:
    GiraffAVR& giraffAVR;
};

typedef struct
{	
    bool is_stopped;
    int32_t mode;
    float a;
    float aw;
    float v;
    float vg;
    float p;
    float vgr;
    int32_t r;
    float lin_speed;
    float ang_speed;
} GiraffState;

/*********************************************************************************/
/**                                                                              */
/** class GiraffManager                                                          */
/**                                                                              */
/** This manages the communication Giraff PC <--> Linux PC <--> Giraff AVR       */
/** It connects with both sides by RS232. It is possible to set the velocity     */
/** of the Giraff by linear velocity and angular velocity. The commands of       */
/** the Giraff PC have priority, only after a timeout without receiving motion   */
/** commands, the manager begins to send motion commands based on the last set   */
/** of linear velocity and angular velocity.                                     */
/**                                                                              */
/*********************************************************************************/
class GiraffManager
{

public:
    /*------------------------------------------------- Constructor -----
    |  Constructor
    |
    |  Parameters:
    |      giraffAVR_device (IN) -- The device name of the Giraff AVR,
    |                               such as "/dev/ttyUSB0"	|
    |      maxLinearVelocity (IN) -- Maximum linear velocity in m/s
    |      maxAngularVelocity (IN) -- Maximum angular velocity in rad/s
    |
    | Comments:
    |     Cconnections will be stablished in the constructor, if some
    |     error happens, a GiraffManagerException will be throw.
    *-------------------------------------------------------------------*/
    GiraffManager(const std::string& giraffAVR_device, int32_t controller_mode, float maxLinVel, float maxAngVel, float acc_lin, float acc_ang, float vgr, std::string battery_technology, float battery_design_capacity, std::string battery_serial_number);
    /*------------------------------------------------- Destructor -----
    |  Destructor
    |
    |  Comments: The connections will be closed
    *-------------------------------------------------------------------*/
    ~GiraffManager();
    /*------------------------------------------------- setVelocity -----
    |  setVelocity
    |
    |  Purpose:
    |	This function sets the linear and angular velocity to be used
    |	when the Giraff PC doesn't have the control.
    |
    |  Parameters:
    |	linear (IN) -- linear velocity in m/s
    |	angular (IN) -- angular velocity in rad/s
    |
    |  Return:
    |	The internal AVR state (mode,a,v,vg,p) for this command
    *-------------------------------------------------------------------*/
    GiraffState setVelocity(float linear, float angular);
    void filterVelocity(float &linearVelocity, float &angularVelocity);
    void set_avr_connection();
    bool check_avr_connection();
    void flush_serial_interface();
    void restart_serial_interface();
    /*------------------------------------------------- getIMD -----
    |  getIMD
    |
    |  Purpose:
    |       This function gets the incremental move distance (IMDL,IMDR)
    |       directly from the AVR
    |
    |  Parameters:
    |	IMDL (OUT) -- Incremental move distance for left wheel in meters
    |	IMDR (OUT) -- Incremental move distance for right wheel in meters
    |
    |
    *-------------------------------------------------------------------*/
    void getIMD(float& imdl, float& imdr);
    void getAMD(float& imdl, float& imdr);
    void get2DOdometry(double &pos_x, double &pos_y, double &yaw, double &lin_vel, double &ang_vel);

    /*--------- getStalk -----
    |  Purpose:
    |       get the current stalk (h) value from AVR
    |  Parameters:
    |  Returns:
    |     h value (0 - 1000)
    *-------------------------------------------------------------------*/
    int getStalk();

    /*--------- setStalk -----
    |  Purpose:
    |       This function sets the stalk height position to the specified value
    |  Parameters:
    |       height -- (float) the desired head's height [meters] w.r.t. its initiial position.
    |       height -- (int) the desired head's height [0-1000]
    *-------------------------------------------------------------------*/
    void setStalk(float height);
    void setStalk(int h_value);

    /*---------- incStalk -----
    |  Purpose:
    |       This function increments the stalk height position 10 steps
    |  Returns:
    |      The current stalk height position (0 - 1000)
    *-------------------------------------------------------------------*/
    int incStalk();

    /*---------- decStalk -----
    |  Purpose:
    |       This function decrements the stalk height position 10 steps
    |  Returns:
    |      The current stalk height position (0 - 1000)
    *-------------------------------------------------------------------*/
    int decStalk();

    /*------------ setTilt -----
    |  Purpose:
    |       This function sets the head tilt to the specified value (rad).    |
    |  Parameters:
    |       tilt -- the desired tilt angle, in radians.
    *-------------------------------------------------------------------*/
    void setTilt(float tilt);

    /*----------- getTilt -----
    |  Purpose:
    |       This function gets the current head tilt in radians
    |  Returns:
    |       The current tilt angle, in radians.
    *-------------------------------------------------------------------*/
    float getTilt();

    /*------------ incTilt -----
    |  Purpose:
    |       This function increments the head tilt angle 0.02 radians
    |  Returns:
    |      The current head tilt angle
    *-------------------------------------------------------------------*/
    float incTilt();

    /*---------- decTilt -----
    |  Purpose:
    |       This function decrements the head tilt angle 0.02 radians
    |  Returns:
    |      The current head tilt angle
    *-------------------------------------------------------------------*/
    float decTilt();


    /*------------------------------------------------- isStopped -----
    |  isStopped
    |
    |  Purpose:
    |       This function determines if the robot is stopped
    |
    |  Parameters:
    |
    |  Returns:
    |      True -> The robot is stopped
    |      False -> The robot is moving
    |
    *-------------------------------------------------------------------*/
    bool isStopped();


    /*------------------------------------------------- getBattery-----
    |  getBattery
    |
    |  Purpose:
    |       get the battery level of the robot
    |
    |  Parameters:
    |      batteryLevel (OUT): The battery level
    |
    |  Returns:
    |     true -> battery level was obtained
    |     false -> battery level cannot be obtained yet
    |
    |
    |
    |
    *-------------------------------------------------------------------*/
    bool getBattery(float& batteryLevel);
//    void getGiraffBatteryData(sensor_msgs::BatteryState &bat);
    bool get_button_data(u_int32_t &red_button, u_int32_t &green_button, u_int32_t &dial);
private:

    GiraffAVR giraffAVR;                //Comms with AVR controller
    GiraffState state;

    int32_t controllerMode;
    float maxLinearVelocity;
    float maxAngularVelocity;
    float lin_acceleration;
    float ang_acceleration;
    float virtualGR;

    //std::string batt_technology, batt_serial_number;
    float batt_design_capacity;

    //ros::Time t_last_velocity_command;
    double last_linearVelocity, last_angularVelocity;    

    void updateState(int32_t mode, float a, float vgr, float v, float vg, float p, int32_t r, float lin_speed, float ang_speed);
    int32_t calculateEncoderIncrement(int32_t previous, int32_t current);
    unsigned int string_to_unsigned_int(std::string in_str);
};





//------------------------------------------- INLINE FUNCTIONS -------------------------------------------------

inline GiraffManager::GiraffManager(const std::string& giraffAVR_device, int32_t controller_mode, float maxLinVel, float maxAngVel, float acc_lin, float acc_ang, float vgr, std::string battery_technology, float battery_design_capacity, std::string battery_serial_number):
    giraffAVR(giraffAVR_device),
    controllerMode(controller_mode),
    maxLinearVelocity(maxLinVel),
    maxAngularVelocity(maxAngVel),
    lin_acceleration(acc_lin),
    ang_acceleration(acc_ang),
   // virtualGR(vgr),
    batt_technology(battery_technology),
    batt_design_capacity(battery_design_capacity),
    batt_serial_number(battery_serial_number)
{
    //Keep state updated
    state.is_stopped = true;
    state.mode = controllerMode;    //set controller mode (0 or 2)
    state.a = lin_acceleration;     //linear acceleration
    state.aw = ang_acceleration;    //angular acceleration
    state.vgr = virtualGR;          //virtual gear ratio
    state.v = 0.0;                  //linear speed
    state.vg = 0.0;                 //angular speed
    state.p = 0.0;                  //(positive-negative) movement

    // Connect to AVR
    set_avr_connection();

    //init time of velocity command sent
    t_last_velocity_command = ros::Time::now();
    last_linearVelocity = 0.0;
    last_angularVelocity = 0.0;
}


void GiraffManager::set_avr_connection()
{
    ROS_INFO("[giraff_ros_driver] Connecting to Giraff AVR");
    if (!giraffAVR.open())
    {
        ROS_ERROR("[giraff_ros_driver] ERROR- Cannot open AVR port....Exiting");
        throw GiraffManagerException(giraffAVR.getLastError());
    }


    // Set Controller mode, stop the robots, and reset wheel encoders to 0 (mandatory for odometry estimation)
    if (controllerMode == 0)
    {
        // This mode operates with V, VG and R parameters
        ROS_INFO("[giraff_ros_driver] Setting Initial State - Controller mode = 0");
        if (!giraffAVR.setInt32("enc0",0) ||
            !giraffAVR.setInt32("enc1",0) ||
            !giraffAVR.setInt32("mode",controllerMode) ||
            !giraffAVR.setFloat("a",lin_acceleration) ||
            !giraffAVR.setFloat("vgr",virtualGR) ||
            !giraffAVR.setFloat("v",0.0) ||
            !giraffAVR.setFloat("vg",0.0) ||
            !giraffAVR.setInt32("r",0) ||
            !giraffAVR.setFloat("p",0))
        {
            throw GiraffManagerException(giraffAVR.getLastError());
        }
    }
    else
    {
        if (controllerMode != 2)
        {
            ROS_INFO("[giraff_ros_driver] Controller mode unknow, setting default mode to 2");
            controllerMode = 2;
            state.mode = controllerMode;
        }

        ROS_INFO("[giraff_ros_driver] Setting Initial State - Controller mode = 2");
        if (!giraffAVR.setInt32("enc0",0) ||
            !giraffAVR.setInt32("enc1",0) ||
            !giraffAVR.setInt32("mode",controllerMode) ||
            !giraffAVR.setFloat("a",lin_acceleration) ||
            !giraffAVR.setFloat("vgr",0) ||
            !giraffAVR.setFloat("v",0.0) ||
            !giraffAVR.setFloat("vg",0.0) ||
            !giraffAVR.setFloat("p",0))
        {
            throw GiraffManagerException(giraffAVR.getLastError());
        }
    }
}


inline GiraffManager::~GiraffManager()
{
    if (giraffAVR.isOpen())
    {
        giraffAVR.setInt32("mode",0);
        giraffAVR.setFloat("a",0);
        giraffAVR.setFloat("v",0);
        giraffAVR.setFloat("vg",0);
        giraffAVR.setFloat("vgr",0);
        giraffAVR.setFloat("p",0);

        giraffAVR.close();  //Close AVR connection!!
    }
}

void GiraffManager::flush_serial_interface()
{
    if (!giraffAVR.isOpen())
        ROS_INFO("[Giraff_ros_driver] Cannot flush serial port (is it oppened?)");
    else
        giraffAVR.flush_serial_interface();
}


bool GiraffManager::check_avr_connection()
{
    return giraffAVR.isOpen();
}


void GiraffManager::restart_serial_interface()
{
    if (giraffAVR.isOpen())
    {
        giraffAVR.setInt32("mode",0);
        giraffAVR.setFloat("a",0);
        giraffAVR.setFloat("v",0);
        giraffAVR.setFloat("vg",0);
        giraffAVR.setFloat("vgr",0);
        giraffAVR.setFloat("p",0);
        giraffAVR.close();  //Close AVR connection!!
        usleep(1000000);   //micro-seconds
    }

    set_avr_connection();   //start serial communication
}


// ----------------------------------------------------------//
//                      FILTER VELOCITY                      //
// ----------------------------------------------------------//
//Filter speeds to ensure correct accelerations, max and min values
void GiraffManager::filterVelocity(float &linearVelocity, float &angularVelocity)
{
    //1. Avoid values too small (close to 0 - since the motors cannot execute them)
    if (linearVelocity<0.001 && linearVelocity>-0.001)
        linearVelocity = 0.0;

    if (angularVelocity<0.001 && angularVelocity>-0.001)
        angularVelocity = 0.0;

    // Allow turning with low linear speeds (bug in the controller board?)
    if (linearVelocity<0.1 && linearVelocity>-0.1 && (angularVelocity>0.1 || angularVelocity<-0.1) )
        linearVelocity = 0.0;

    //2. Check speed ranges (max-min)
    if (linearVelocity > maxLinearVelocity)
        linearVelocity = maxLinearVelocity;
    else if (linearVelocity < -maxLinearVelocity)
        linearVelocity = -maxLinearVelocity;

    if (angularVelocity > maxAngularVelocity)
        angularVelocity = maxAngularVelocity;
    else if (angularVelocity < -maxAngularVelocity)
        angularVelocity = -maxAngularVelocity;

    //3. Check speed increment is below max_accelerations
    ros::Time t_now = ros::Time::now();
    ros::Duration t_inc = t_now - t_last_velocity_command;
    float max_speed_inc_allowed, max_ang_speed_inc_allowed;
    //lin
    if (t_inc.toSec() < 1.0)
        max_speed_inc_allowed = lin_acceleration * t_inc.toSec();
    else
        max_speed_inc_allowed = lin_acceleration * 0.5; //to avoid high values after a long time whitout receiving commands

    double inc_desired_speed = linearVelocity - last_linearVelocity;
    if ( fabs(inc_desired_speed) > max_speed_inc_allowed )
    {
        if (inc_desired_speed > 0)
            linearVelocity = last_linearVelocity + max_speed_inc_allowed;
        else
            linearVelocity = last_linearVelocity - max_speed_inc_allowed;
    }

    //ang
    if (t_inc.toSec() < 1.0)
        max_ang_speed_inc_allowed = ang_acceleration * t_inc.toSec();
    else
        max_ang_speed_inc_allowed = ang_acceleration * 0.5;  //to avoid high values after a long time whitout receiving commands

    double inc_desired_ang_speed = angularVelocity - last_angularVelocity;
    if ( fabs(inc_desired_ang_speed) > max_ang_speed_inc_allowed )
    {
        if (inc_desired_ang_speed > 0)
            angularVelocity = last_angularVelocity + max_ang_speed_inc_allowed;
        else
            angularVelocity = last_angularVelocity - max_ang_speed_inc_allowed;
    }
    //ROS_INFO("[Giraff_ros_driver] Tinc = %.3f desired_inc_speed = %.3f (max is %.3f)", t_inc.toSec(), fabs(inc_desired_ang_speed), max_ang_speed_inc_allowed);

    //update variables
    t_last_velocity_command = t_now;
    last_linearVelocity = linearVelocity;
    last_angularVelocity = angularVelocity;
}


// ----------------------------------------------------------//
// ---------------------- SET VELOCITY ----------------------//
// ----------------------------------------------------------//
inline GiraffState GiraffManager::setVelocity(float linearVelocity, float angularVelocity)
{
    //Filter speeds (accelerations, max and min values)
    filterVelocity(linearVelocity, angularVelocity);

    /*
     * TWM  - Turn while moving (set vg, cdp, vgr, v, p, p)
     * VG   - Max virtual gear ratio between wheels
     * CDP(m_clothoidDecelerationPoint) - Clothoid deceleration point
     * VGR(m_virtualGearRateOfChange) - Virtual gear change rate
     * V(m_vel) - Max velocity
     * P(m_pos) - Position to go to
     * GRD(m_currentGearRatioDifferential) - Current gear ratio differential. 0 is straight
     */

    float v,vg,p,vgr,cdp,r;
    bool use_twm = true;    // Use Turn While Moving (TWM)
    float cdp_cte = 1.01;   // Clothoid deceleration point (Cte). Setting this just above the value of p, but a lower value does also seem to work.


    // CONTROLLER MODE 2 (V,W) (Old Giraffs)
    if (controllerMode == 2)
    {
        // Stop?
        if (linearVelocity>-0.001 && linearVelocity<0.001 && angularVelocity>-0.001 && angularVelocity<0.001)
        {
            if (!giraffAVR.setFloat("v",0.0) || !giraffAVR.setFloat("vg",0.0) || !giraffAVR.setFloat("p",0.0))
                throw GiraffManagerException(giraffAVR.getLastError());
            
            // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
            updateState(state.mode, state.a, state.vgr, 0.0, 0.0, 0.0, state.r, linearVelocity, angularVelocity);
        }
        else
        {    
            if (linearVelocity >= 0.001)  // Positive V
            {
                v = linearVelocity;
                p = 1;
            }
            else if (linearVelocity<=-0.001)  //Negative V
            {
                v = -linearVelocity;    //v is always positive, the sign goes into P
                p = -1;
            }
            else    //Guard, should never enter here,... but just in case! XD
            {
                v = 0;
                p = 1;
            }

            //Angular speed. In this mode 2, vg = w, so things are easy
            vg = angularVelocity*180/PI;    //vg in degrees

            //Send new command to AVR (v, vg, p)
            //ROS_INFO("[Giraff_ros_driver] setting v=%.3f , vg=%.3f , p=%.3f",v,vg,p);
            if (!giraffAVR.setFloat("v",v) || !giraffAVR.setFloat("vg",vg) || !giraffAVR.setFloat("p",p))
                throw GiraffManagerException(giraffAVR.getLastError());

            // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
            updateState(state.mode, state.a, state.vgr, v, vg, p, state.r, linearVelocity, angularVelocity);
        }
    }

    // CONTROLLER MODE 0 (v,vg,r,p,twm)
    else if (controllerMode == 0)
    {
        //------------------------
        //      Stop?
        //------------------------
        if (linearVelocity>-0.001 && linearVelocity<0.001 && angularVelocity>-0.001 && angularVelocity<0.001)
        {
            /* From Camanio Tech Support
            * set mode 0
            * set a lin_acc
            * set r 50 (Straight line - Not sure why they don't just set r to 1)
            * set p 0
            * set vgr 0
            */
            if ( !giraffAVR.setInt32("mode",0) || !giraffAVR.setFloat("a",lin_acceleration) || !giraffAVR.setInt32("r",50) || !giraffAVR.setFloat("vgr",0.0) || !giraffAVR.setFloat("p",0.0))
                throw GiraffManagerException(giraffAVR.getLastError());
         
            // Keep updated the State variable (same acc and mode, but all speeds to 0.0)
            // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
            updateState(0, lin_acceleration, 0.0, state.v, state.vg, 0.0, 50, linearVelocity, angularVelocity);
        }
        else
        {
            // In this Operation mode we have 3 types of movement : turn_in_place, straight and curve
            
            //------------------------
            // (A) TURN IN PLACE (v=0)
            //------------------------
            if (linearVelocity == 0.0)
            {
                /* From Camanio Tech Support
                *  set mode 0
                *  set a Angular Acceleration.
                *  set v Speed of rotation.
                *  set r 0  // Rotate
                *  set p 90 // Rotation in degrees.
                *  set vgr = VGR
                */
                r = 0;  // Rotate
                v = angularVelocity > 0.0 ? (angularVelocity*DISTANCE_B_WHEELS/2) : (-angularVelocity*DISTANCE_B_WHEELS/2);   // Speed of rotation
                p = angularVelocity > 0.0 ? -90.0 : 90.0;                                                                     // Rotation in degrees
                vgr = virtualGR;

                // ROS_INFO("[Giraff_ros_driver] ROTATION - setting r=%i , v=%.3f , p=%.3f",r,v,p);
                if ( !giraffAVR.setInt32("mode",0) || !giraffAVR.setFloat("a",ang_acceleration) || !giraffAVR.setFloat("vgr",vgr) || !giraffAVR.setFloat("v",v) || !giraffAVR.setInt32("r",r) || !giraffAVR.setFloat("p",p) )
                    throw GiraffManagerException(giraffAVR.getLastError());

                // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
                updateState(0, ang_acceleration, vgr, v, state.vg, p, r, linearVelocity, angularVelocity);
            }
            

            //------------------------
            // (B) Straight line (w=0)
            //------------------------
            else if (fabs(angularVelocity) <= 0.05)
            {
                if (use_twm)
                {
                    // Send command Turn While Moving (twm)
                    // vg = 0 (not relevant for straight movement)
                    // cdp = +/-CDP according to forward/backward
                    // vgr = 0
                    // v = max desired lin_speed
                    // p = +/-1 according to forward/backward
                    vg = 0;
                    cdp = linearVelocity > 0.0 ? cdp_cte : -cdp_cte;
                    vgr = 0.0;
                    v = fabs(linearVelocity);
                    p = linearVelocity > 0.0 ? 1.0 : -1.0;

                    if (giraffAVR.setTwm(vg, cdp, vgr, v, p) == false)
                        throw GiraffManagerException(giraffAVR.getLastError());

                    // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
                    updateState(0, lin_acceleration, vgr, v, vg, p, state.r, linearVelocity, angularVelocity);
                }
                else
                {
                    // mode = 0
                    // vg = 0 (not relevant for straight movement)
                    // cdp = +/-CDP according to forward/backward
                    // vgr = 0
                    // v = max desired lin_speed
                    // p = 1
                    // r = 1 (or >1)
                    // p = distance in metters
                    r = 50;
                    v = fabs(linearVelocity);
                    p = linearVelocity > 0.0 ? 1.0 : -1.0;
                    vgr = virtualGR;

                    // ROS_INFO("[Giraff_ros_driver] STRAIGHT - setting r=%i , v=%.3f , p=%.3f",r,v,p);
                    if (!giraffAVR.setInt32("mode",0) || !giraffAVR.setFloat("a",lin_acceleration) || !giraffAVR.setFloat("vgr",vgr) || !giraffAVR.setFloat("v",v) || !giraffAVR.setInt32("r",r) || !giraffAVR.setFloat("p",p))
                        throw GiraffManagerException(giraffAVR.getLastError());

                    // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
                    updateState(0, lin_acceleration, vgr, v, state.vg, p, r, linearVelocity, angularVelocity);
                }
            }
            

            //------------------------
            // (C) Curved movement (most of the cases)
            //------------------------
            else
            {
                if (use_twm)
                {
                    // Send command Turn While Moving (twm)
                    // vg = (|W|*dist_wheels) / (2*V - |W|*dist_wheels)
                    // cdp = +/-CDP according to forward/backward
                    // vgr = +/-VGR according to left/right
                    // v = max desired lin_speed
                    // p = +/- 1 according to forward/backward
                    v = fabs(linearVelocity);
                    float vg_cte = (fabs(angularVelocity)*DISTANCE_B_WHEELS) / (2*v - fabs(angularVelocity)*DISTANCE_B_WHEELS);
                    vg = angularVelocity > 0.0 ? -vg_cte : vg_cte;
                    cdp = linearVelocity > 0.0 ? cdp_cte : -cdp_cte;
                    vgr = angularVelocity > 0.0 ? -virtualGR : virtualGR;
                    p = linearVelocity > 0.0 ? 1.0 : -1.0;

                    if (giraffAVR.setTwm(vg, cdp, vgr, v, p) == false)
                        throw GiraffManagerException(giraffAVR.getLastError());

                    // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
                    updateState(0, lin_acceleration, vgr, v, vg, p, state.r, linearVelocity, angularVelocity);
                }
                else
                {
                    r = 100;
                    v = fabs(linearVelocity);
                    float vg_cte = (fabs(angularVelocity)*DISTANCE_B_WHEELS) / (2*v - fabs(angularVelocity)*DISTANCE_B_WHEELS);

                    // set signs according to V and W
                    if (linearVelocity>0.0 && angularVelocity>0.0) //front + turn_left
                    {
                        vg = -vg_cte;
                        vgr = -vgr;
                        p = 1.0;
                        cdp = cdp_cte;
                    }
                    else if (linearVelocity>0 && angularVelocity<0) //front + turn_right
                    {
                        vg = vg_cte;
                        p = 1.0;
                        cdp = cdp_cte;
                    }
                    else if (linearVelocity<0 && angularVelocity<0)  //back + turn_right
                    {
                        vg = -vg_cte;
                        p = -1.0;
                        cdp = -cdp_cte;
                    }
                    else //back + turn_left
                    {
                        vg = vg_cte;
                        vgr = -vgr;
                        p = -1.0;
                        cdp = -cdp_cte;
                    }

                    //Send new command to AVR
                    // ROS_INFO("[Giraff_ros_driver] CRUVED - setting r=%i , vgr=%.3f , v=%.3f , vg=%.3f , p=%.3f",r,vgr,v,vg,p);
                    if (use_twm)
                    {
                        // Send command Turn While Moving (twm)
                        if (giraffAVR.setTwm(vg, cdp, vgr, v, p) == false)
                            throw GiraffManagerException(giraffAVR.getLastError());
                    }
                    else
                    {
                        // This code produces some errors  and cause short stops of the motors
                        if ( !giraffAVR.setInt32("mode",0) || !giraffAVR.setFloat("vgr",vgr) || !giraffAVR.setFloat("v",v) || !giraffAVR.setFloat("vg",vg) || !giraffAVR.setInt32("r",r) || !giraffAVR.setFloat("p",p))
                            throw GiraffManagerException(giraffAVR.getLastError());
                    }
                    // updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
                    updateState(state.mode, state.a, vgr, v, vg, p, r, linearVelocity, angularVelocity);
                }





            }
        }
    }
    
    // CONTROLLER MODE?
    else
    {
        ROS_ERROR("[Giraff Driver] Unknown Operation Mode!");
    }

    // Return current state
    return state;
}


// Update STATE of the Controller Params to keep a record
// updateState(mode, a, vgr, v, vg, p, r, lin_speed, ang_speed)
inline void GiraffManager::updateState(int32_t mode, float a, float vgr, float v, float vg, float p, int32_t r, float lin_speed, float ang_speed)
{
    #ifdef _MONITOR_ACTUAL_AVR_PARAMETERS_
    giraffAVR.writeCommand("get mode\r",mode);
    giraffAVR.writeCommand("get a\r",a);
    giraffAVR.writeCommand("get vgr\r",vgr);
    giraffAVR.writeCommand("get v\r",v);
    giraffAVR.writeCommand("get vg\r",vg);
    giraffAVR.writeCommand("get p\r",p);
    giraffAVR.writeCommand("get r\r",r);
    #endif

    //Update internal State var
    state.mode = mode;
    state.a = a;
    state.vgr = vgr;
    state.v = v;
    state.vg = vg;
    state.p = p;
    state.r = r;
    state.lin_speed = lin_speed;
    state.ang_speed = ang_speed;
}



std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


inline void GiraffManager::get2DOdometry(double &pos_x, double &pos_y, double &yaw, double &lin_vel, double &ang_vel)
{
    // Read bulk data from Giraff controller to get imdl and imdr
    //-----------------------------------------------------------
    std::string sbulk;
    if (!giraffAVR.writeCommand("get bulk_data\r",sbulk))
    {
        throw GiraffManagerException(giraffAVR.getLastError());
    }

    //BULK: cang:F*00000000,cdis:F*00000000,gvr:F*00000000,tilt_angle_from_home:F*F051B83D,imdl:F*00000000,imdr:F*00000000,cvg:F*00000000,mode:I*02000000,
    std::vector<std::string> tokens = split(sbulk,',');
    std::string simdl, simdr;
    simdl = tokens[4];
    simdr = tokens[5];
    //ROS_INFO("%s %s",simdl.c_str(),simdr.c_str());

    //1. Remove characters from string = imdl:F*00000000
    size_t pos = simdl.find("*");
    simdl.erase(0, pos+1);
    simdr.erase(0, pos+1);

    //2. Pairwise-swap the string (see Giraff Controller API)
    for(size_t i=0, middle=simdl.size()/2, size=simdl.size(); i<middle; i+=2 )
    {
        std::swap(simdl[i], simdl[size - i- 2]);
        std::swap(simdl[i+1], simdl[size -i - 1]);

        std::swap(simdr[i], simdr[size - i- 2]);
        std::swap(simdr[i+1], simdr[size -i - 1]);
    }

    //3. IEEE 754 : HEX to Float
    unsigned int x;
    std::stringstream ss;
    ss << std::hex << simdl;
    ss >> x;
    float imdl = reinterpret_cast<float&>(x);
    unsigned int y;
    std::stringstream ss2;
    ss2 << std::hex << simdr;
    ss2 >> y;
    float imdr = reinterpret_cast<float&>(y);

    //Get odometry from IDML and IMDR
    double incOrientation,incCenterOfRobot;
    incCenterOfRobot = (imdr + imdl)/2;
    incOrientation = (imdr-imdl)/DISTANCE_B_WHEELS;
    yaw = yaw + incOrientation;
    //normalize yaw in [-pi,pi]
    while (yaw <= -3.14159) yaw += 2*3.14159;
    while (yaw > 3.14159) yaw -= 2*3.14159;
    pos_x = pos_x + incCenterOfRobot*cos( yaw - 0.5*incOrientation );
    pos_y = pos_y + incCenterOfRobot*sin( yaw - 0.5*incOrientation );



    //Get linear and angular speeds
    //-----------------------------
    std::string sv, sw;
    if (!giraffAVR.writeCommand("get v\r",sv) || !giraffAVR.writeCommand("get vg\r",sw))
    {
        throw GiraffManagerException(giraffAVR.getLastError());
    }


    //1. Remove characters from string = v:F*00000000
    pos = sv.find("*");
    sv.erase(0, pos+1);
    sw.erase(0, pos+1);
    sv = sv.substr(0,8);
    sw = sw.substr(0,8);

    //ROS_INFO("v=%s",sv.c_str());
    //ROS_INFO("w=%s",sw.c_str());

    //2. Pairwise-swap the string (see Giraff Controller API)
    for(size_t i=0, middle=sv.size()/2, size=sv.size(); i<middle; i+=2 )
    {
        std::swap(sv[i], sv[size - i- 2]);
        std::swap(sv[i+1], sv[size -i - 1]);

        std::swap(sw[i], sw[size - i- 2]);
        std::swap(sw[i+1], sw[size -i - 1]);
    }

    //3. IEEE 754 : HEX to Float
    unsigned int xv;
    std::stringstream ssv;
    ssv << std::hex << sv;
    ssv >> xv;
    lin_vel = reinterpret_cast<float&>(xv);

    unsigned int xw;
    std::stringstream ssw;
    ssw << std::hex << sw;
    ssw >> xw;
    ang_vel = reinterpret_cast<float&>(xw);
    ang_vel = ang_vel*3.14159/180;                  //get in radians
}



///* Get battery information from Giraff robot */
//void GiraffManager::getGiraffBatteryData(sensor_msgs::BatteryState &bat)
//{
//    //Giraff provides battery data sequentially due to buffer restrictions (see manual), so we need multiple readigns to complete a battery_msg
//    // PARAMETER_IDENTIFIERS
//    //      B:F#_ battery voltage in volts
//    //      b:F#_ battery current in amps
//    //      t:F#_ battery temperature in degrees Celsius
//    //      D:F#_ dock voltage in volt
//    //      d:F#_ dock current in amps
//    //      W:F#_ watt/s drawn from the battery (0=fully charged)
//    bool msg_complete = false;
//    bool has_v = false, has_i = false, has_ch = false;
//
//    while (!msg_complete)
//    {
//        // Chager data format: S:S#[Charger Status value],[Parameter identifier]:[Value identifier]#[Parameter value] U#[CRC checksum][CR (carriage return character)]
//        std::string bat_data;
//        if (!giraffAVR.writeCommand("get charger_data\r",bat_data))
//            throw GiraffManagerException(giraffAVR.getLastError());
//
//        // Split string into fields
//        //--------------------------
//        // Charger Status value: S:S#ABCD --> D=0 fast charging, D=1 Trickle charging, D=2 Error, D=3 Init, D=4 Not docked
//        size_t pos = bat_data.find(",");
//        std::string charge_status = bat_data.substr(0, pos);
//        bat_data.erase(0, pos+1);
//        double ch = atof( &(*charge_status.rbegin()) );
//        //Parameter identifier + value identifier
//        pos = bat_data.find("#");
//        std::string parameter = bat_data.substr(0, pos);
//        bat_data.erase(0, pos+1);
//        //Value
//        pos = bat_data.find(" ");
//        std::string parameter_value = bat_data.substr(0, pos);
//        bat_data.erase(0, pos+1);
//
//        //Pairwise-swap the string of the vlaue
//        for(size_t i=0, middle=parameter_value.size()/2, size=parameter_value.size(); i<middle; i+=2 )
//        {
//            std::swap(parameter_value[i], parameter_value[size - i- 2]);
//            std::swap(parameter_value[i+1], parameter_value[size -i - 1]);
//        }
//        //IEEE 754 : HEX to Float
//        unsigned int x;
//        std::stringstream ss;
//        ss << std::hex << parameter_value;
//        ss >> x;
//        float parameter_float = reinterpret_cast<float&>(x);
//
//        //Fill Battery-msg
//        if (!strcmp(parameter.c_str(),"B:F") )  //parameter is battery voltage (V)
//        {
//            bat.voltage = parameter_float;
//            has_v = true;
//        }
//        else if (!strcmp(parameter.c_str(),"b:F") )  //parameter is battery current (A)
//        {
//            bat.current = parameter_float;
//            has_i = true;
//        }
//        else if (!strcmp(parameter.c_str(),"W:F") )  //parameter is battery watt/s
//        {
//            //Get approx State of Charge (battery %)
//            float BATT_WATT_SEC = -327360.0;       // Cte from Camanio (related to the battery used on Giraff robots)
//            float soc = (BATT_WATT_SEC - parameter_float) / BATT_WATT_SEC;
//            bat.percentage = soc;                   // Charge percentage: 0 to 1 range  (If unmeasured NaN)
//
//            // Set Charger status
//            if (parameter_float == 0.0)
//                bat.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;   //fully_charged = 4
//            else if (ch<2)
//                bat.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;   //Charging = 1
//            else
//                bat.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;   //Not Charging = 3
//            has_ch = true;
//        }
//
//        //check if we have all fields
//        if (has_v and has_i and has_ch)
//            msg_complete = true;
//    }
//
//    //Complete battery msgs
//    bat.header.stamp = ros::Time::now();
//    bat.header.frame_id = "/base_link";
//    bat.present = true;
//    bat.serial_number = batt_serial_number;
//    bat.design_capacity = batt_design_capacity;    // Capacity in Ah (design capacity)  (If unmeasured NaN)
//    //Technology
//    if (batt_technology == "NIMH")
//        bat.power_supply_technology = 1;
//    else if (batt_technology == "LION")
//        bat.power_supply_technology = 2;
//    else if (batt_technology == "LIPO")
//        bat.power_supply_technology = 3;
//    else if (batt_technology == "LIFE")
//        bat.power_supply_technology = 4;
//    else if (batt_technology == "NICD")
//        bat.power_supply_technology = 5;
//    else if (batt_technology == "LIMN")
//        bat.power_supply_technology = 6;
//    else
//        bat.power_supply_technology = 0;    //Unknown
//    //Todo - It'd be nice to do this properly. We'd need to run some studies
//    bat.charge = NAN;             // Current charge in Ah  (If unmeasured NaN)
//    bat.capacity = NAN;           // Capacity in Ah (last full capacity)  (If unmeasured NaN)
//}
//
//





// *********************  OTHER FUNCTIONS NOT COMMONLY USED ********************* //

inline int32_t GiraffManager::calculateEncoderIncrement(int32_t previous, int32_t current)
{
	// Encoder range: -32768 to 32767

	int32_t a = previous + 32768; // From 0 to 65535
	int32_t b = current  + 32768; // From 0 to 65535

	int32_t option1 = b-a; 
	int32_t option2 = (option1>0)?(-65536 + option1):(65536 + option1);

	return abs(option1)<abs(option2) ? option1:option2;
}


inline void GiraffManager::getIMD(float& imdl, float& imdr)
{
	static int32_t prev_enc0=0,prev_enc1=0;
	int32_t enc0,enc1,inc_enc0,inc_enc1;

	if (!giraffAVR.writeCommand("get enc0\r",enc0) ||
        !giraffAVR.writeCommand("get enc1\r",enc1))
    {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	inc_enc0 = calculateEncoderIncrement(prev_enc0,enc0);
	inc_enc1 = calculateEncoderIncrement(prev_enc1,enc1);
	prev_enc0 = enc0;
	prev_enc1 = enc1;
	
    if (inc_enc0==0 && inc_enc1==0)
    {
        state.is_stopped = true;
		imdl = 0.0;
		imdr = 0.0;
	}
    else
    {
        state.is_stopped = false;
		imdl = (float)inc_enc0/2002.0594;
		imdr = (float)inc_enc1/2002.0594;
	} 
}


inline void GiraffManager::getAMD(float& imdl, float& imdr)
{
    int32_t enc0,enc1;

    //Read encoders (absolute accumulation)
    if (!giraffAVR.writeCommand("get enc0\r",enc0) ||
        !giraffAVR.writeCommand("get enc1\r",enc1))
    {
        throw GiraffManagerException(giraffAVR.getLastError());
    }

    if (enc0==0 && enc1==0)
    {
        state.is_stopped = true;
        imdl = 0.0;
        imdr = 0.0;
    }
    else
    {
        state.is_stopped = false;
        imdl = (float)enc0/2002.0594;
        imdr = (float)enc1/2002.0594;
    }
}


//****** STALK *****************//
inline int GiraffManager::getStalk()
{
    int32_t stalk;
    if (!giraffAVR.writeCommand("get h\r",stalk))
        throw GiraffManagerException(giraffAVR.getLastError());

    return stalk;   // range [0 - 1000]
}

inline void GiraffManager::setStalk(float height)
{
    //Ensure is in range
    height = std::max(height, (float) MIN_STALK_HEIGHT);
    height = std::min(height, (float) MAX_STALK_HEIGHT);

    //Discretize
    int32_t h_cmd = round((height - MIN_STALK_HEIGHT)/(MAX_STALK_HEIGHT-MIN_STALK_HEIGHT)*1000);

    //Set
    if (!giraffAVR.setInt32("h", h_cmd))
		throw GiraffManagerException(giraffAVR.getLastError());
}

inline void GiraffManager::setStalk(int h_value)
{
    //Ensure is in range
    int h;
    h = std::max(h_value, 0);
    h = std::min(h_value, 1000);

    //Set
    if (!giraffAVR.setInt32("h", h))
        throw GiraffManagerException(giraffAVR.getLastError());
}



inline int GiraffManager::incStalk()
{
    // Get current stalk (h) value
	int32_t stalk;
    if (!giraffAVR.writeCommand("get h\r",stalk))
        throw GiraffManagerException(giraffAVR.getLastError());
	
    // Increase value
    int32_t oldStalk = stalk;
    stalk += 5;
	
    // check ranges
    if (stalk>1000)
        stalk = 1000;

    if (stalk!=oldStalk)
    {
        // Set new h value
        if (!giraffAVR.setInt32("h",stalk))
            throw GiraffManagerException(giraffAVR.getLastError());
	}
    // return current value
	return stalk;
}

inline int GiraffManager::decStalk()
{
    // get current h value
    int32_t stalk;
    if (!giraffAVR.writeCommand("get h\r",stalk))
        throw GiraffManagerException(giraffAVR.getLastError());

    // decrease
    int32_t oldStalk = stalk;
    stalk -= 5;
    if (stalk<0)
        stalk = 0;

    // Set
    if (stalk!=oldStalk)
    {
        if (!giraffAVR.setInt32("h",stalk))
            throw GiraffManagerException(giraffAVR.getLastError());
    }

    // Return current value
    return stalk;
}



//****** TILT *****************//
inline void GiraffManager::setTilt(float tilt)
{
    //Check max-min values
    if (fabs(tilt) > 2*M_PI/3.0)
    {
        bool sign = std::signbit(tilt);
        tilt = (1-2*sign) * (2*M_PI/3.0);
    }

    //Set tilt
    if (!giraffAVR.setFloat("tilt_angle_from_home",tilt))
        throw GiraffManagerException(giraffAVR.getLastError());
}


inline float GiraffManager::getTilt()
{
    float tilt;
    if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt))
        throw GiraffManagerException(giraffAVR.getLastError());

    return tilt;
}


inline float GiraffManager::incTilt()
{
    float tilt;
    if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt))
        throw GiraffManagerException(giraffAVR.getLastError());

    float oldTilt = tilt;
    tilt += 0.1;

    if (tilt > 2.0943935)     //(2*M_PI/3.0)
        tilt = 2.0943935;

    if (tilt!=oldTilt)
    {
        if (!giraffAVR.setFloat("tilt_angle_from_home",tilt))
            throw GiraffManagerException(giraffAVR.getLastError());
    }

    return tilt;
}

inline float GiraffManager::decTilt()
{
    float tilt;
    if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt))
        throw GiraffManagerException(giraffAVR.getLastError());

    float oldTilt = tilt;
    tilt -= 0.1;

    if (tilt<0.0872664) {
        tilt = 0.0872664;
    }
    if (tilt!=oldTilt)
    {
        if (!giraffAVR.setFloat("tilt_angle_from_home",tilt))
            throw GiraffManagerException(giraffAVR.getLastError());
    }
    return tilt;
}



// Deprecated. Use instead the most complete getGiraffBatteryData
inline bool GiraffManager::getBattery(float &batteryLevel)
{
	const static float c = -327360.0f;
	std::string response;
	if (!giraffAVR.writeCommand("get charger_data\r",response)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	size_t p1 = response.find_first_of(',');
	if (p1	== std::string::npos)  {
		return false;
	}
	p1++;
	if (p1 == response.size()) {
		return false;
	}
	if (response[p1]!='W') {
		return false;
	}
	p1+=2;	

	size_t p2 = response.find_first_of(' ',p1);
	if (p2	== std::string::npos)  {
		return false;
	}
	float ws;
	if (!GiraffAVR::toFloat(response.substr(p1,p2-p1),ws)) {
		return false;
	}
	
	batteryLevel = std::abs((c-ws)/c); 
	
	return true;
}



inline bool GiraffManager::isStopped()
{
    return state.is_stopped;
}


inline bool GiraffManager::get_button_data(u_int32_t &red_button, u_int32_t &green_button, u_int32_t &dial)
{
    std::string response;
    if (!giraffAVR.writeCommand("get button_data\r",response)) {
        throw GiraffManagerException(giraffAVR.getLastError());
    }
    //ROS_INFO("[giraff_get_buttons_data] Reading is: %s", response.c_str());

    // response format is: but0:[value],but1:[value],dial:[value],
    //                     but0:I*29000000,but1:I*07000000,dial:I*00000000,

    // Red_button = but0
    // Value is the number of times the button has been pressed since starting time (acumulative count)
    // Always positive numbers
    size_t pos = response.find(",");
    std::string red_str = response.substr(0, pos);
    response.erase(0, pos+1);
    //Remove characters from string => name:I*00000000
    pos = red_str.find("*");
    red_str.erase(0, pos+1);
    red_button = static_cast<u_int32_t>(string_to_unsigned_int(red_str));


    // Green_button = but1
    // Value is the number of times the button has been pressed since starting time (acumulative count)
    // Always positive numbers
    pos = response.find(",");
    std::string green_str = response.substr(0, pos);
    response.erase(0, pos+1);
    //Remove characters from string => name:I*00000000
    pos = green_str.find("*");
    green_str.erase(0, pos+1);
    green_button = static_cast<u_int32_t>(string_to_unsigned_int(green_str));

    //dial (volumen control)
    // Value is the position of the dial with respect starting position
    // Positive numbers (unsigned int32) from [0 to uint_32_max)
    pos = response.find(",");
    std::string dial_str = response.substr(0, pos);
    //Remove characters from string => name:I*00000000
    pos = dial_str.find("*");
    dial_str.erase(0, pos+1);
    dial = static_cast<u_int32_t>(string_to_unsigned_int(dial_str));

    return true;
}


unsigned int GiraffManager::string_to_unsigned_int(std::string in_str)
{
    //Pairwise-swap the string (see Giraff Controller API)
    for(size_t i=0, middle=in_str.size()/2, size=in_str.size(); i<middle; i+=2 )
    {
        std::swap(in_str[i], in_str[size - i- 2]);
        std::swap(in_str[i+1], in_str[size -i - 1]);
    }

    //3. HEX_str to Uint
    unsigned int y = std::stoul(in_str, nullptr, 16);
    /* C mode:
    unsigned int x;
    std::stringstream ss;
    ss << std::hex << in_str;   //Set HEX as the numeric base
    ss >> x;
    */
    return y;
}

//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
