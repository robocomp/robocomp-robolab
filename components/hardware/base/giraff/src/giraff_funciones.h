#ifndef GIRAFF_FUNCIONES_H
#define GIRAFF_FUNCIONES_H

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <string>
//#include <math.h>
#include "giraff_serial.h"
#include "specificworker.h"

#define PI 3.14159265359
#define DISTANCE_B_WHEELS 0.467  
#define CLK_TCK 1000.0

typedef struct
{	
    bool is_stopped;
    int32_t mode;
    float a;
    float aw;
    float v;
    float vg;
    float lin_speed;
    float ang_speed;
    float tilt;
} GiraffState;

typedef struct
{	
    std::time_t timeStamp; //Timestamp
    float voltage;
    float current;
    float percentage;
    int32_t power_supply_status;
    
}BatteryState;

typedef struct
{	
    float v;
    float vg;
    float x;
    float z;
    float alpha;

}Odometria;

typedef struct
{	
    u_int32_t Rojo;
    u_int32_t Verde;
    u_int32_t Dial;

}Botones;


GiraffPC giraff("/dev/ttyS1");
GiraffAVR giraffavr("/dev/ttyS1");
     

//------------INICIO DE COMUNICACION
bool iniciarAvr();

//------------OBTENCIÓN DE DATOS DE LA BATERIA

bool getGiraffBatteryData(BatteryState &bat);

bool getGiraffOdometria(Odometria &Od);

//----TILT---

void setTilt(float tilt);

float getTilt();

float incTilt();

float decTilt();

//--------------BOTONES

bool get_button_data(Botones &Botones);




//------------------------FUNCIONES------------------------------------//
//
//

bool iniciarAvr(){        
    
    int controllerMode=0;
    float lin_acceleration=0.2;
    float ang_acceleration=0.4;
    float virtualGR=20;

    if(giraffavr.open() && giraff.open())
    {
        std::cout<<"INICIO CORRECTO AVR."<<std::endl;
    }
    
    usleep(3000000);

    giraffavr.setFloat("tilt_angle_from_home",0.7);

    //ESTABLECIMIENTO VALORES INICIALES A 0.

    if (!giraffavr.setInt32("enc0",0) ||
        !giraffavr.setInt32("enc1",0) ||
        !giraffavr.setInt32("mode",controllerMode) ||
        !giraffavr.setFloat("a",lin_acceleration) ||
        !giraffavr.setFloat("vgr",virtualGR) ||
        !giraffavr.setFloat("v",0.0) ||
        !giraffavr.setFloat("vg",0.0) ||
        !giraffavr.setInt32("r",50) ||
        !giraffavr.setFloat("p",0))
    {
        return false;
    }
    else
    {
        return true;
    }
}



bool getGiraffBatteryData(BatteryState &bat){
    //Giraff provides battery data sequentially due to buffer restrictions (see manual), so we need multiple readigns to complete a battery_msg
    // PARAMETER_IDENTIFIERS
    //      B:F#_ battery voltage in volts
    //      b:F#_ battery current in amps
    //      t:F#_ battery temperature in degrees Celsius
    //      D:F#_ dock voltage in volt
    //      d:F#_ dock current in amps
    //      W:F#_ watt/s drawn from the battery (0=fully charged)

    bool msg_complete = false;
    bool has_v = false, has_i = false, has_ch = false;
    std::string batt_technology;
   

    while (!msg_complete)
    {
        // Chager data format: S:S#[Charger Status value],[Parameter identifier]:[Value identifier]#[Parameter value] U#[CRC checksum][CR (carriage return character)]
        std::string bat_data;
        if (!giraffavr.writeCommand("get charger_data\r",bat_data))
            std::cout<<"Error al obtener datos de batería"<<std::endl;
        // Split string into fields
        //--------------------------
        // Charger Status value: S:S#ABCD --> D=0 fast charging, D=1 Trickle charging, D=2 Error, D=3 Init, D=4 Not docked
        size_t pos = bat_data.find(",");
        std::string charge_status = bat_data.substr(0, pos);
        bat_data.erase(0, pos+1);
        double ch = atof( &(*charge_status.rbegin()) );
        //Parameter identifier + value identifier
        pos = bat_data.find("#");
        std::string parameter = bat_data.substr(0, pos);
        bat_data.erase(0, pos+1);
        //Value
        pos = bat_data.find(" ");
        std::string parameter_value = bat_data.substr(0, pos);
        bat_data.erase(0, pos+1);

        //Pairwise-swap the string of the vlaue
        for(size_t i=0, middle=parameter_value.size()/2, size=parameter_value.size(); i<middle; i+=2 )
        {
            std::swap(parameter_value[i], parameter_value[size - i- 2]);
            std::swap(parameter_value[i+1], parameter_value[size -i - 1]);
        }
        //IEEE 754 : HEX to Float
        unsigned int x;
        std::stringstream ss;
        ss << std::hex << parameter_value;
        ss >> x;
        float parameter_float = reinterpret_cast<float&>(x);

        //Fill Battery-msg
        if (!strcmp(parameter.c_str(),"B:F") )  //parameter is battery voltage (V)
        {
            bat.voltage = parameter_float;
            has_v = true;
        }
        else if (!strcmp(parameter.c_str(),"b:F") )  //parameter is battery current (A)
        {
            bat.current = parameter_float;
            has_i = true;
        }
        else if (!strcmp(parameter.c_str(),"W:F") )  //parameter is battery watt/s
        {
            //Get approx State of Charge (battery %)
            float BATT_WATT_SEC = -327360.0;       // Cte from Camanio (related to the battery used on Giraff robots)
            float soc = (BATT_WATT_SEC - parameter_float) / BATT_WATT_SEC;
            bat.percentage = soc;                   // Charge percentage: 0 to 1 range  (If unmeasured NaN)

            // Set Charger status
            if (parameter_float == 0.0)
                bat.power_supply_status = 1;   //fully_charged = 4
            else if (ch<2)
                bat.power_supply_status = 2;   //Charging = 1
            else
                bat.power_supply_status = 3;   //Not Charging = 3
            has_ch = true;
        }

        //check if we have all fields
        if (has_v and has_i and has_ch ){
            msg_complete = true;
            time(&bat.timeStamp);
        }
            
    }

    std::cout<<"BATERÍA"<<std::endl;
    std::cout<<"Timestamp:"<<bat.timeStamp<<std::endl;
    std::cout<<"Voltaje:"<<bat.voltage<<std::endl;
    std::cout<<"Corriente:"<<bat.current<<std::endl;
    std::cout<<"Porcentaje:"<<bat.percentage*100<<"%"<<std::endl;
    std::cout<<"Estado Carga = 1/2/3 (FULL/cargando/no conectado):"<<bat.power_supply_status<<std::endl;

    
    return msg_complete;
}




void SetSpeedBase(float adv, float rot)
{   
    float lin_acceleration = 0.3;
    float ang_acceleration = 0.3;
    float r = 50;
    float linearVelocity = adv*0.7;
    float angularVelocity = -rot;
    float v,vg,p,vgr,cdp;
    float maxLinearVelocity = 0.7;
    float maxAngularVelocity = 0.7;
    float virtualGR = 20;
    float cdp_cte = 1.05;
    

    //-----ESTABLECER STOP--------

    if (linearVelocity>-0.001 && linearVelocity<0.001 && angularVelocity>-0.001 && angularVelocity<0.001)
    {
        if (!giraffavr.setFloat("v",0.0) || !giraffavr.setFloat("vg",0.0) || !giraffavr.setFloat("p",0.0))
        {
            std::cout<<"Error V=0"<<std::endl;
        }
    }

    //////////      FILTRO DE VELOCIDAD        /////////////77

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



    //IMPLEMENTACION II: giraff mode 0   
    


    // Clothoid deceleration point (Cte). Setting this just above the value of p, but a lower value does also seem to work.
    //------------------------
    //      Stop?
    //------------------------
    if (linearVelocity>-0.01 && linearVelocity<0.01 && angularVelocity>-0.01 && angularVelocity<0.01)
    {
        if ( !giraffavr.setInt32("mode",0) || !giraffavr.setFloat("a",lin_acceleration) || !giraffavr.setInt32("r",0) || !giraffavr.setFloat("vgr",0.0) || !giraffavr.setFloat("p",0.0))
        {
            std::cout<< "Error al parar"<<std::endl;
        }    
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
            p = angularVelocity > 0.0 ? -90.0 : 90.0;        /////CAMBIADO, ORIGINALMENTE +-90                            // Rotation in degrees
            vgr = virtualGR*angularVelocity;

            // ROS_INFO("[Giraff_ros_driver] ROTATION - setting r=%i , v=%.3f , p=%.3f",r,v,p);
            if ( !giraffavr.setInt32("mode",0) || !giraffavr.setFloat("a",ang_acceleration) || !giraffavr.setFloat("vgr",vgr) || !giraffavr.setFloat("v",v) || !giraffavr.setInt32("r",r) || !giraffavr.setFloat("p",p) )
            {
                std::cout<<"Error al girar"<<std::endl;
            }

        std::cout<<"GIRANDO"<<std::endl;
        }
        //------------------------
        // (B) Straight line (w=0)
        //------------------------
        else if (fabs(angularVelocity) <= 0.05)
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
            vgr = 0;
            //vgr=0;
                    
            if (!giraffavr.setInt32("mode",0) || !giraffavr.setFloat("a",lin_acceleration) || !giraffavr.setFloat("vgr",vgr) || !giraffavr.setFloat("v",v) || !giraffavr.setInt32("r",r) || !giraffavr.setFloat("p",p))
            {
                std::cout<<"Error en linea recta"<<std::endl;
            }            
            std::cout<<"LINEA RECTA"<<std  ::endl; 
        }    
        //------------------------
        // (C) Curved movement (most of the cases)
        //------------------------
        else
        {
                
            vgr = virtualGR*fabs(angularVelocity);   
            vgr = virtualGR;  
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
            
            if ( !giraffavr.setInt32("mode",0) || !giraffavr.setFloat("vgr",vgr) || !giraffavr.setFloat("a",lin_acceleration) || !giraffavr.setFloat("v",v) || !giraffavr.setFloat("vg",vg) || !giraffavr.setInt32("r",r) || !giraffavr.setFloat("p",p))
            {
                std::cout<<"error al mandar comando al AVR"<<std::endl;
            }
                
        }
    }

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


bool getGiraffOdometria(Odometria &Od)
{
    // Read bulk data from Giraff controller to get imdl and imdr
    //-----------------------------------------------------------
    std::string sbulk;

    if (!giraffavr.writeCommand("get bulk_data\r",sbulk))
    {
        std::cout<<"Error en bulk_data"<<std::endl;
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


    Od.alpha = Od.alpha + incOrientation;
    //normalize yaw in [-pi,pi]
    while (Od.alpha <= -3.14159) Od.alpha += 2*3.14159;
    while (Od.alpha > 3.14159) Od.alpha -= 2*3.14159;
    Od.x = Od.x + incCenterOfRobot*cos( Od.alpha - 0.5*incOrientation );
    Od.z = Od.z + incCenterOfRobot*sin( Od.alpha - 0.5*incOrientation );



    //Get linear and angular speeds
    //-----------------------------
    std::string sv, sw, p;
    if (!giraffavr.writeCommand("get v\r",sv) || !giraffavr.writeCommand("get vg\r",sw))
    {
        std::cout<<"Error en la adquisición de datos"<<endl;    
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

        std::swap(p[i], p[size - i- 2]);
        std::swap(p[i+1], p[size -i - 1]);
    }

    //3. IEEE 754 : HEX to Float
    unsigned int xv;
    std::stringstream ssv;
    ssv << std::hex << sv;
    ssv >> xv;
    Od.v = reinterpret_cast<float&>(xv);

    unsigned int xw;
    std::stringstream ssw;
    ssw << std::hex << sw;
    ssw >> xw;
    Od.vg = reinterpret_cast<float&>(xw);
    Od.vg = Od.vg*3.14159/180;                  //get in radians


    return true;
}


//****** TILT *****************//

void setTilt(float tilt)
{
    //Check max-min values
    if (fabs(tilt) > 2*M_PI/3.0)
    {
        bool sign = std::signbit(tilt);
        tilt = (1-2*sign) * (2*M_PI/3.0);
    }
    //Set tilt
    if (!giraffavr.setFloat("tilt_angle_from_home",tilt)){
    std::cout<<"Error en establecimiento del angulo."<<std::endl;
    }
    
}


float getTilt()
{
    float tilt;
    if (!giraffavr.writeCommand("get tilt_angle_from_home\r",tilt))
     //   throw GiraffManagerException(giraffAVR.getLastError());

    return tilt;
}


float incTilt()
{
    float tilt;
    if (!giraffavr.writeCommand("get tilt_angle_from_home\r",tilt))
        std::cout<<"Error en lectura del angulo"<<std::endl;

    float oldTilt = tilt;
    tilt += 0.1;

    if (tilt > 2.0943935)     //(2*M_PI/3.0)
        tilt = 2.0943935;

    if (tilt!=oldTilt)
    {
        if (!giraffavr.setFloat("tilt_angle_from_home",tilt))
            std::cout<<"Error en establecimiento del angulo"<<std::endl;
    }

    return tilt;
}

float decTilt()
{
    float tilt;
    if (!giraffavr.writeCommand("get tilt_angle_from_home\r",tilt))
        std::cout<<"Error en obtención del angulo"<<std::endl;

    float oldTilt = tilt;
    tilt -= 0.1;

    if (tilt<0.0872664) {
        tilt = 0.0872664;
    }
    if (tilt!=oldTilt)
    {
        if (!giraffavr.setFloat("tilt_angle_from_home",tilt))
            std::cout<<"Error en establecimiento del angulo"<<std::endl;
    }
    return tilt;
}


unsigned int string_to_unsigned_int(std::string in_str)
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

bool get_button_data(Botones &Botones)
{
    std::string response;
    if (!giraffavr.writeCommand("get button_data\r",response)) {
        std::cout<<"Error en botones"<<std::endl;
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
    Botones.Rojo = static_cast<u_int32_t>(string_to_unsigned_int(red_str));


    // Green_button = but1
    // Value is the number of times the button has been pressed since starting time (acumulative count)
    // Always positive numbers
    pos = response.find(",");
    std::string green_str = response.substr(0, pos);
    response.erase(0, pos+1);
    //Remove characters from string => name:I*00000000
    pos = green_str.find("*");
    green_str.erase(0, pos+1);
    Botones.Verde = static_cast<u_int32_t>(string_to_unsigned_int(green_str));

    //dial (volumen control)
    // Value is the position of the dial with respect starting position
    // Positive numbers (unsigned int32) from [0 to uint_32_max)
    pos = response.find(",");
    std::string dial_str = response.substr(0, pos);
    //Remove characters from string => name:I*00000000
    pos = dial_str.find("*");
    dial_str.erase(0, pos+1);
    Botones.Dial = static_cast<u_int32_t>(string_to_unsigned_int(dial_str));

    return true;
}


#endif // GIRAFF_FUNCIONES_H
