/***********************************************************************/
/**                                                                    */
/** giraff_serial.h                                                    */
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
/***********************************************************************/


#ifndef _GIRAFF_SERIAL_H_
#define _GIRAFF_SERIAL_H_

// Activate this to show some debug information about the communications Linux PC <--> Giraff AVR
#define _GIRAFF_AVR_DEBUG_

// Activate this to show some debug information about the communications Linux PC <--> Giraff PC
#define _GIRAFF_PC_DEBUG_

// Activate only if you are using the old USB connections
//#define _OLD_USB_CONNECTIONS_

#include <sstream>
#include <string>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <unistd.h>

//-- HEADERS ----------------------------------------------------------------------------

/*********************************************************************************/
/**                                                                              */
/** class SerialInterface                                                        */
/**                                                                              */
/** An abstract class implementing general methods for reading/writing the RS232 */            
/** serial interface                                                             */
/**                                                                              */
/*********************************************************************************/
class SerialInterface
{
	
	public:
	/*------------------------------------------------- Constructor -----
	|  Constructor
	|
	|  Parameters:
	|      devicename (IN) -- The device name, as "/dev/ttyUSB0"
	|      hardware_flow_control (IN) -- If true, flow control by hardware 
	|                                    will be activated at open method
	|   
	|  Comments:
	|     The serial port will be opened when the program calls the open
	|     method.
	|  
	*-------------------------------------------------------------------*/
	SerialInterface(const std::string& devicename, bool hardware_flow_control);
	/*------------------------------------------------- Destructor ----- 
	|  Destructor
	|
	|  Purpose: free the memory and close the serial port if it's open 
	*-------------------------------------------------------------------*/
	virtual ~SerialInterface();
	/*------------------------------------------------- open -----
	|  Function open
	|
	|  Purpose:  Open the serial port and configure it at B115200 and 8N1
	|            The flow control by hardware will be activated if
	|            hardware_flow_control has been passed to the constructor
	|
	|  Parameters:
	|
	|  Returns:  true if the serial port has been opened successfully
	|            false if some error has occurred
	*-------------------------------------------------------------------*/
	virtual bool open();
	/*------------------------------------------------- close -----
	|  Function close
	|
	|  Purpose:  Close the serial port
	|
	|  Parameters:
	|
	|  Returns:  true if the serial port has been closed successfully
	|            false if some error has occurred
	*-------------------------------------------------------------------*/
	virtual bool close();
	/*------------------------------------------------- isOpen -----
	|  Function isOpen
	|
	|  Purpose:  Check if serial port is open
	|
	|  Parameters:turtlebot ros
	|
	|  Returns:  true if the serial port is open
	|            false if the serial port is closed
	*-------------------------------------------------------------------*/
	bool isOpen();
	/*------------------------------------------------- getLastError -----
	|  Function getLastError
	|
	|  Purpose:  Get an human readable description of the last error 
	|	         if some function returns false 
	|
	|  Parameters:
	|
	|  Returns:  string describing the last error
	*-------------------------------------------------------------------*/
	const std::string& getLastError();
        void flush_serial_interface();
	
	protected:
	bool setRTS(bool rts);
	bool getDSR(bool& dsr);	
	bool getRTS(bool& rts);
	bool incomingBytes(int& bytes);
	bool write(const std::string& str);	
	int read(char* buffer, int buffer_size);
	void setLastError(const std::string& lastError);
	virtual void closeNow();
	
	private:
	std::string devicename; 
	bool hardware_flow_control;
	int fd;
	std::string lastError;

};

class GiraffAVRMonitor
{
    public:
        GiraffAVRMonitor() {}
        virtual ~GiraffAVRMonitor() {}
        virtual void notifyCommand(const std::string& command) {}
        virtual void notifyResponse(const std::string& response) {}
} defaultMonitor;

/*********************************************************************************/
/**                                                                              */
/** class GiraffAVR                                                              */
/**                                                                              */
/** A class implementing methods for communicating with the Giraff AVR           */           
/**                                                                              */
/*********************************************************************************/
class GiraffAVR : public SerialInterface
{
	public:
	/*------------------------------------------------- Constructor -----
	|  Constructor
	|
	|  Parameters:
	|      devicename (IN) -- The serial port to the Giraff AVR, as "/dev/ttyUSB0"
	|
	|  Comments:
	|     The communication will be initiated when the program calls the open
	|     method. 
	|  
	*-------------------------------------------------------------------*/
    //GiraffAVR(const std::string& devicename, GiraffAVRMonitor& monitor = defaultMonitor);
    GiraffAVR(const std::string& devicename);
	/*------------------------------------------------- Destructor -----
	| Destructor
	*-------------------------------------------------------------------*/
	virtual ~GiraffAVR();
	/*------------------------------------------------- open -----
	|  Function open
	|
	|  Purpose:  Connect at 115200 and 8N1. 
	|            After connection, flag RTS will be activated to start communication. 
	|            Then, we read the welcome message from the Giraff AVR.
	|            The hardware link must be as follows:
	| 
	|        (Linux PC)         (Giraff AVR)
	|                
	|            CD                 CD
	|            Rx-----------------Tx
	|            Tx-----------------Rx
	|            GND----------------GND
	|            RTS----------------DSR
	|
	|  Parameters:
	|
	|  Returns:  true if the Giraff AVR has been connected successfully
	|            false if some error has occurred
	|  
	*-------------------------------------------------------------------*/
	virtual bool open();
	/*------------------------------------------------- close -----
	|  Function close
	|
	|  Purpose:  Set RTS to 0 and close the connection with the Giraff AVR
	|
	|  Parameters:
	|
	|  Returns:  true if no errors
	|            false if some error has occurred
	*-------------------------------------------------------------------*/		
	virtual bool close();
	/*------------------------------------------------- getWelcomeMessage ---
	|  Function getWelcomeMessage
	|
	|  Purpose:  Get the welcome message obtained from the Giraff AVR at the
	|            connection process in open method.
	|
	|  Parameters:
	|
	|  Returns:  The welcome message,  as "# Giraf 1.1.112p, 2011-02-20\r\nOK >\r\n".
	|            It is two lines ended by "\r\n",the first one start always with "# Giraf"
	|            and the second is always "OK >\r\n"
	*-------------------------------------------------------------------*/			
	const std::string& getWelcomeMessage();
	/*------------------------------------------------- writeCommand ---
	|  Function writeCommand
	|
	|  Purpose:  Send a command to the Giraff AVR and get the response. 
	|            It is a polymorphic function, depending of the type of
	|            response, it returns a string (raw response), a float 
	|            or an integer.
	|
	|  Parameters:
	|      command (IN) -- A command to send, it must end with '\r'.
	|      response (IN/OUT) -- The response of the Giraff AVR:
	|                           (1) If the type of response is std::string, then
	|                           it will be the raw response formed by one
	|                           or more lines ended by "\r\n", 
	|                           the last line is always "OK >\r\n".
	|                           (2) If the type of response is float, then the raw
	|                           response will be parsed to a float if possible.
	|                           (3) If the type of response is int32_t, then the 
	|                           raw response will be parsed to an int32_t if
	|                           possible.
	|
	|  Returns:  true if no errors.
	|            false if some error has occurred
	*-------------------------------------------------------------------*/			
	bool writeCommand(const std::string& command, std::string& response);
	bool writeCommand(const std::string& command, float& response);
	bool writeCommand(const std::string& command, int32_t& response);
	/*------------------------------------------------- setFloat ---
	|  Function setFloat
	|
	|  Purpose:  This is a wrapper over writeCommand in order to set
	|            a float variable. setFloat(x,value) is translated to
	|            "set x value\r"
	|
	|  Parameters:
	|      variable (IN) -- Float variable to be set.
	|         value (IN) -- Value for the variable.
	|
	|  Returns:  true if no errors.
	|            false if some error has occurred
	*-------------------------------------------------------------------*/				
	bool setFloat(const std::string& variable, float value);
    bool setTwm(float vg, float cdp, float vgr, float v, float p);

	/*------------------------------------------------- setInt32 ---
	|  Function setInt32
	|
	|  Purpose:  This is a wrapper over writeCommand in order to set
	|            an int32 variable. setInt32(x,value) is translated to
	|            "set x value\r"
	|
	|  Parameters:
	|      variable (IN) -- Int32 variable to be set.
	|         value (IN) -- Value for the variable.
	|
	|  Returns:  true if no errors.
	|            false if some error has occurred
	*-------------------------------------------------------------------*/		
	bool setInt32(const std::string& variable, int32_t value); 
	
	/*------------------------------------------------- toFloat ---
	|  Function toFloat
	|
	|  Purpose:  Convert a string from the AVR with the format F*XXXXXXXX or F#XXXXXXXX to a float
	|
	|  Parameters:
	|      str (IN) -- The string to convert 
	|      value (OUT) -- The obtained float.
	|
	|  Returns:  true if successful.
	|            
	*-------------------------------------------------------------------*/		
	static bool toFloat(const std::string& str, float& value);
	private:
    //GiraffAVRMonitor& monitor;
	std::string welcomeMessage;
	static bool getByte(char high, char low, uint8_t& b);
	bool readResponse(std::string& response, bool takeItEasy=false);
	virtual void closeNow();
	

};

/*************************************************************************************/
/**                                                                                  */
/** class Action                                                                     */
/**                                                                                  */
/** An abstract class in order to implement the Strategy Design Pattern              */
/** http://en.wikipedia.org/wiki/Strategy_pattern                                    */
/**                                                                                  */
/*************************************************************************************/

class Action
{
	public:
	Action() {}
	virtual ~Action() {}
	virtual bool doAction() {return true;};
} defaultAction;


/************************************************************************************/
/**                                                                                 */
/** class GiraffPC                                                                  */
/**                                                                                 */
/** A class implementing methods for communicating with the Giraff PC               */           
/**                                                                                 */
/** The methods of this class should be called by the main program in this fashion: */
/**                                                                                 */
/**  1.-   open                                                                     */
/**  2.-   LOOP:                                                                    */
/**  2.1.-  checkDSR                                                                */
/**  2.2.-  readCommand                                                             */
/**  2.3.-  ... process command, maybe you want to ask the Giraff AVR...            */
/**  2.4.-  writeResponse                                                           */
/**  3.-   close                                                                    */
/**                                                                                 */
/**                                                                                 */
/************************************************************************************/

class GiraffPC : public SerialInterface
{
	public:
	/*------------------------------------------------- Constructor -----
	|  Constructor
	|
	|  Parameters:
	|      devicename (IN) -- The serial port to the Giraff PC, as "/dev/ttyUSB1"
	|
	|  Comments:
	|     The communication will be initiated when the program calls the open
	|     method. There is a state "initiated" (communications are allowed)
	|     or "not initiated" (communications are not allowed). 
	|     At the beginning, it's "not initiated".
	|  
	*-------------------------------------------------------------------*/
	GiraffPC(const std::string& devicename);
	/*------------------------------------------------- Destructor -----
	| Destructor
	*-------------------------------------------------------------------*/
	virtual ~GiraffPC() {}
	/*------------------------------------------------- open -----
	|  Function open
	|
	|  Purpose:  Connect at 115200 and 8N1. 
	|            The hardware link must be as follows: 
	|
	|        (Giraff PC)         (Linux PC)
	|                
	|            CD                 CD
	|            Rx-----------------Tx
	|            Tx-----------------Rx
	|            GND----------------GND
	|            DTR----------------RTS
	|
	|  Parameters:
	|
	|  Returns:  true if the Giraff PC has been connected successfully,
	|            false if some error has occurred
	|  
	*-------------------------------------------------------------------*/	
	virtual bool open();
	/*------------------------------------------------- checkRTS ---
	|  Function checkRTS
	|
	|  Purpose: 
	|       (1) If the state is "not initiated" and RTS is active, 
	|           we wait 1 second and send the welcome message,
	|           the state is now "initiated"
	|       (2) If the state is "initiated" and RTS is not active, the
	|           state is now "not initiated"
	|
	|  Parameters:
	|      welcomeMessage (IN) -- The welcomeMessage, as 
	|                             "# Giraf 1.1.112p, 2011-02-20\r\nOK >\r\n"
	|                             to be sended in case (1)
	|      action (IN) -- An action to do in case (1) before waiting 1 second
	|
	|  Returns:  true if no errors
	|            false if some error has occurred
	*-------------------------------------------------------------------*/			
	bool checkRTS(const std::string& welcomeMessage, Action& action = defaultAction);
	/*------------------------------------------------- checkRTS ---
	|  Function checkDSR (used ifdef _OLD_USB_CONNECTIONS_)
	|
	|  Purpose: 
	|       (1) If the state is "not initiated" and DSR is active, 
	|           we wait 1 second and send the welcome message,
	|           the state is now "initiated"
	|       (2) If the state is "initiated" and DSR is not active, the
	|           state is now "not initiated"
	|
	|  Parameters:
	|      welcomeMessage (IN) -- The welcomeMessage, as 
	|                             "# Giraf 1.1.112p, 2011-02-20\r\nOK >\r\n"
	|                             to be sended in case (1)
	|      action (IN) -- An action to do in case (1) before waiting 1 second
	|
	|  Returns:  true if no errors
	|            false if some error has occurred
	*-------------------------------------------------------------------*/			
	bool checkDSR(const std::string& welcomeMessage, Action& action = defaultAction);
	/*------------------------------------------------- readCommand ---
	|  Function readCommand
	|
	|  Purpose: Read a command from the Giraff PC if data is available (non-blocking read)
	|
	|  Parameters:
	|      command (IN/OUT) -- A command from the Giraff PC. If data is not available or
	|                          the state is "not initiated", the command will 
	|                          have 0 size. The command ends with '\r'.
	| 
	|
	|  Returns:  true if no errors
	|            false if some error has occurred
	*-------------------------------------------------------------------*/	
	bool readCommand(std::string& command); 
	/*------------------------------------------------- writeResponse ---
	|  Function writeResponse
	|
	|  Purpose: Write a response to the Giraff PC 
	|
	|  Parameters:
	|      response (IN) -- A response to the Giraff PC. If the state 
	|                       is "not initiated", the response will be ignored.
	|                       The response must ends with "\r\n"
	| 
	|
	|  Returns:  true if no errors
	|            false if some error has occurred
	*-------------------------------------------------------------------*/			
	bool writeResponse(const std::string& response);
	
	/*------------------------------------------------- isInitiated ---
	|  Function isInitiated
	|
	|  Purpose: Get the state of the communication
	|
	|  Parameters:
	|
	|  Returns:  true: state is "initiated"
	|            false: state is "not initiated"
	*-------------------------------------------------------------------*/		
	bool isInitiated();

	private:
	bool initiated;
};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)


inline void _printTime() {
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	char day[4], mon[4];
	int wday, hh, mm, ss, year;
	sscanf(ctime((time_t*) &(ts.tv_sec)), "%s %s %d %d:%d:%d %d",day, mon, &wday, &hh, &mm, &ss, &year);
	//std::cout << std::setw(2) << std::setfill(' ') << wday << ' ' << mon << ' ' << year << ' ' << std::setfill('0') << hh << ':' << mm << ':' << ss << '.' << ts.tv_nsec<< ' ';
}


/***********************************/
/** SerialInterface implementation */
/***********************************/

//Init serial interface
inline SerialInterface::SerialInterface(const std::string& devicename, bool hardware_flow_control) : 
devicename(devicename), 
hardware_flow_control(hardware_flow_control),
fd(-1)  //by default it is not open!
{}

//destructor serial interface
inline SerialInterface::~SerialInterface()
{
    if (fd!=-1)     //if not closed then close it
        ::close(fd);
}

inline bool SerialInterface::open()
{
    if (fd!=-1)     //is it alreay open?
    {
        lastError = std::string("[Giraff_ros_driver] Cannot open ") + devicename + std::string(" because it's already open.");
        //std::cout  <<  "[Giraff_ros_driver] " << lastError.c_str() << std::endl;
        return false;
    }

    //Lets try open the serial port
    struct termios attr;
    bool success =  ((fd = ::open(devicename.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) != -1) &&      //open port
                    (fcntl(fd, F_SETFL, 0) != -1) &&            //
                    (tcgetattr(fd, &attr) != -1) &&             //get attributes
                    (cfsetospeed (&attr, B115200) != -1) &&     //default badurate
                    (cfsetispeed (&attr, B115200) != -1) &&     //defaul baudrate
                    (tcflush(fd, TCIOFLUSH) != -1);             //flush serial port on opening

    if (success)
    {
        //std::cout  <<  "[Giraff_ros_driver] Successfully opened AVR on " <<  devicename.c_str() << std::endl;
        attr.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        attr.c_cflag |= (CS8 | CLOCAL | CREAD);
        if (hardware_flow_control)
            attr.c_cflag |= CRTSCTS;
        else
            attr.c_cflag &= ~CRTSCTS;
        attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        attr.c_iflag = 0;
        attr.c_oflag = 0;
        attr.c_cc[VMIN]  = 0;
        attr.c_cc[VTIME] = 1;
        success = (tcsetattr(fd, TCSANOW, &attr) != -1);
    }

    if (!success)
    {
        //std::cout  <<  "[Giraff_ros_driver] Error opening AVR on " <<  devicename.c_str() << std::endl;
        //ROS_ERROR("%s", strerror(errno));
        lastError = std::string(strerror(errno));
        closeNow();
    }

    return success;
}

inline bool SerialInterface::close()
{
    if (fd==-1)
    {
        lastError = std::string("Cannot close ") + devicename + std::string(" because it isn't open.");
        return false;
    }

    bool success = (::close(fd) != -1);

    if (success)
        fd = -1;
    else
        lastError = std::string(strerror(errno));

    return success;
}


void SerialInterface::flush_serial_interface()
{
    usleep(500);
    //ioctl(fd, TCFLSH, 2);       // flush both ports (in and out)
    tcflush(fd,TCIOFLUSH);      //another way to flush
}

inline bool SerialInterface::isOpen()
{
    return fd!=-1;
}

inline const std::string& SerialInterface::getLastError()
{
    return lastError;
}

inline bool SerialInterface::setRTS(bool rts)
{
    int status;
    bool success = (ioctl(fd, TIOCMGET, &status) != -1);
    if (success)
    {
        if (rts) status |= TIOCM_RTS;
        else status &= ~TIOCM_RTS;
        success = (ioctl(fd, TIOCMSET, &status) != -1);
    }
    if (!success)
        lastError = std::string(strerror(errno));
    return success;
}


inline bool SerialInterface::getDSR(bool& dsr)
{
    int s;
    bool success = (ioctl(fd,TIOCMGET,&s) != -1);
    if (!success) {
        lastError = std::string(strerror(errno));
    }
    else {
        dsr = (s & TIOCM_DSR) != 0;
    }
    return success;
}

inline bool SerialInterface::getRTS(bool& rts)
{
    int s;
    bool success = (ioctl(fd,TIOCMGET,&s) != -1);
    if (!success) {
        lastError = std::string(strerror(errno));
    }
    else {
        rts = (s & TIOCM_RTS) != 0;
    }
    return success;
}

inline bool SerialInterface::incomingBytes(int& bytes)
{
    bool success= (ioctl(fd, FIONREAD, &bytes)!=-1);
    if (!success) {
        lastError = std::string(strerror(errno));
    }
    return success;
}

inline bool SerialInterface::write(const std::string& str)
{
    bool success =  (::write (fd, str.c_str(), str.size()) == (int)str.size());
    if (!success) {
        lastError = std::string(strerror(errno));
    }
    return success;
}	

inline int SerialInterface::read(char* buffer, int buffer_size)
{
    int bytes = ::read(fd,buffer,buffer_size);

    if (bytes==-1) {
        lastError = std::string(strerror(errno));
    }
    return bytes;
}

inline void SerialInterface::setLastError(const std::string& lastError)
{
    this->lastError = lastError;
}


inline void SerialInterface::closeNow()
{
    if (fd!=-1)
    {
        ::close(fd);
        fd = -1;
    }
}



/*****************************/
/** GiraffAVR implementation */
/*****************************/

//inline GiraffAVR::GiraffAVR(const std::string& devicename, GiraffAVRMonitor& monitor) :
//SerialInterface(devicename,false),
//monitor(monitor)
//{}

inline GiraffAVR::GiraffAVR(const std::string& devicename) : SerialInterface(devicename,false)
{}

inline GiraffAVR::~GiraffAVR()
{
    if (SerialInterface::isOpen())
        SerialInterface::setRTS(false);
}

inline bool GiraffAVR::open()
{
    if (!SerialInterface::open()) {
        return false;
    }
    welcomeMessage="";
    #ifdef _GIRAFF_AVR_DEBUG_
    _printTime();
    //std::cout<<"Activating RTS->DSR flag to Giraff AVR"<<std::endl;
    #endif
    if (!SerialInterface::setRTS(false)) {
        closeNow();
        return false;
    }
    usleep(5000);

    if (!SerialInterface::setRTS(true)) {
        closeNow();
        return false;
    }
    #ifdef _GIRAFF_AVR_DEBUG_
    _printTime();
    //std::cout<<"Waiting for Giraff AVR welcome message"<<std::endl;
    #endif
    if (!readResponse(welcomeMessage,true)) {
        closeNow();
        return false;
    }
    size_t n = welcomeMessage.find("# Giraf");
    if (n== std::string::npos) {
        setLastError("Invalid welcome message");
        closeNow();
        return false;
    }
    welcomeMessage=welcomeMessage.substr(n,welcomeMessage.size());
    #ifdef _GIRAFF_AVR_DEBUG_
            _printTime();
            //std::cout << "Giraff AVR -> Linux PC  : "<< welcomeMessage.substr(0,welcomeMessage.find_first_of('\r')) << std::endl;
    #endif

    return true;
}

inline void GiraffAVR::closeNow() {
	welcomeMessage="";
	SerialInterface::setRTS(false);
	SerialInterface::closeNow();
}


inline bool GiraffAVR::close()
{
    if (!SerialInterface::setRTS(false))
        return false;
    if (!SerialInterface::close())
        return false;
    welcomeMessage = "";
    return true;
}


inline const std::string& GiraffAVR::getWelcomeMessage()
{
    return welcomeMessage;
}

inline bool GiraffAVR::writeCommand(const std::string& command, std::string& response)
{
    //monitor.notifyCommand(command.substr(0,command.find_first_of('\r')));
    do{
        if (!SerialInterface::write (command)) {
            return false;
        }
        #ifdef _GIRAFF_AVR_DEBUG_
            _printTime();
            //std::cout << "Linux PC   -> Giraff AVR: "<< command.substr(0,command.find_first_of('\r')) << std::endl;
        #endif
        if (!readResponse(response)) {
             return false;
        }
        #ifdef _GIRAFF_AVR_DEBUG_
            _printTime();
            //std::cout << "Giraff AVR -> Linux PC  : "<< response.substr(0,response.find_first_of('\r')) << std::endl;
        #endif

    } while(response.size()==0);
    //monitor.notifyResponse(response.substr(0,response.find_first_of('\r')));
    return true;
}


inline bool GiraffAVR::writeCommand(const std::string& command, float& response)
{
	std::string responseString;
	bool retry;
	
	do{
		retry=false;
		if (!writeCommand(command,responseString)) {
			return false;
		}
		if (responseString.size()!=18 ||
			responseString.find("F*")!=0 ||	
			responseString.substr(responseString.find_first_of('\r')).compare("\r\nOK >\r\n")!=0) {
			retry=true;
			continue;
		}
		union {
			float n;
			uint8_t b[4];
		} bytes;
		int cont=0;
		for(int i=2;i<10;i+=2) {
			if(!getByte(responseString[i],responseString[i+1],bytes.b[cont++])) {
				retry=true;
				break;	
			}
		}
		response = bytes.n;
		
	}while(retry);
	return true;
}


//-----------------------------------------------------//
//          TWM - Turn While Moving                    //
//-----------------------------------------------------//
inline bool GiraffAVR::setTwm(float vg, float cdp, float vgr, float v, float p)
{
    char buffer [50];
    int n;
    n = sprintf(buffer, "set twm %.2f %.2f %.2f %.2f %.2f 0\r", vg, cdp, vgr, v, p);

    std::string command(buffer);
    std::string response;
    return writeCommand(command,response);
}

inline bool GiraffAVR::setFloat(const std::string& variable, float value)
{
    //WTF??
    /*
    if (value > -0.009 && value< 0.009) {
            value=0;
    }
    */
    std::ostringstream os;
    os<<value;
    std::string command= "set "+variable+" "+os.str()+"\r";
    std::string response;
    return writeCommand(command,response);
	
}
inline bool GiraffAVR::setInt32(const std::string& variable, int32_t value)
{
    std::ostringstream os;
    os<<value;
    std::string command= "set "+variable+" "+os.str()+"\r";
    std::string response;
    return writeCommand(command,response);
}

inline bool GiraffAVR::writeCommand(const std::string& command, int32_t& response)
{
    std::string responseString;
    bool retry;

    do{
        retry=false;
        if (!writeCommand(command,responseString)) {
            return false;
        }
        if (responseString.size()!=18 ||
            responseString.find("I*")!=0 ||
            responseString.substr(responseString.find_first_of('\r')).compare("\r\nOK >\r\n")!=0) {
            retry=true;
            continue;
        }
        union {
            int32_t n;
            uint8_t b[4];
        } bytes;
        int cont=0;
        for(int i=2;i<10;i+=2) {
            if(!getByte(responseString[i],responseString[i+1],bytes.b[cont++])) {
                retry=true;
                break;
            }
        }
        response = bytes.n;

    }while(retry);
    return true;
}

inline bool GiraffAVR::toFloat(const std::string& str, float& value)
{
    if (str.size()!=10 || str[0]!='F') {
        return false;
    }
    union {
        float n;
        uint8_t b[4];
    } bytes;
    int cont=0;
    for(int i=2;i<10;i+=2) {
        if(!getByte(str[i],str[i+1],bytes.b[cont++])) {
            return false;
        }
    }
    value = bytes.n;
    return true;
}


inline bool GiraffAVR::getByte(char high, char low, uint8_t& b)
{
    char x;
    b=0;
    for (int i=0;i<2;i++) {
        b = b<<4;
        x = (i==0)?high:low;
        if (x>='0' && x<='9') {
            b |= (x-'0');
        }
        else if (x>='A' && x<='F') {
            b |= (x-'A')+10;
        }
        else if (x>='a' && x<='f') {
            b |= (x-'a')+10;
        }
        else {
            return false;
        }
    }
    return true;
}

inline bool GiraffAVR::readResponse(std::string& response, bool takeItEasy)
{
    static char buffer[257];
    int bytes;
    response="";
    do {
        bytes = SerialInterface::read(buffer,256);
        if (bytes==-1) {
            response="";
            return false;
        }
        if (bytes==0 && !takeItEasy) {
            response="";
            return true;
        }
        buffer[bytes]=0;
        response+=buffer;
    }while(response.find("OK >\r\n")==std::string::npos);
    return true;
}



/******************************/
/** GiraffPC implementation   */
/******************************/

inline GiraffPC::GiraffPC(const std::string& devicename) :
SerialInterface(devicename,false),
initiated(false)
{}


inline bool GiraffPC::checkDSR(const std::string& welcomeMessage, Action& action)
{
	bool dsr;
	if (!SerialInterface::getDSR(dsr)) {
		return false;
	}
	if (!initiated && dsr) {
		if (!action.doAction()) {
			setLastError("Failed initial action in checkDSR");
			return false;
		}
		sleep(1);
		if (!SerialInterface::write(welcomeMessage)) {
			return false;
		}
		initiated=true;
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Giraff PC  -> Linux PC  : DTR up" << std::endl;
		#endif	
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Linux PC   -> Giraff PC : "<< welcomeMessage.substr(0,welcomeMessage.find_first_of('\r')) << std::endl;
		#endif	
	}
	else if (initiated && !dsr) {
		initiated=false;
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Giraff PC  -> Linux PC  : DTR down"<<std::endl;
		#endif	
	}
	return true;
}


inline bool GiraffPC::checkRTS(const std::string& welcomeMessage, Action& action)
{
	bool rts;
	if (!SerialInterface::getRTS(rts)) {
		return false;
	}
	if (!initiated && rts) {
		if (!action.doAction()) {
			setLastError("Failed initial action in checkRTS");
			return false;
		}
		sleep(1);
		if (!SerialInterface::write(welcomeMessage)) {
			return false;
		}
		initiated=true;
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Giraff PC  -> Linux PC  : DTR up" << std::endl;
		#endif	
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Linux PC   -> Giraff PC : "<< welcomeMessage.substr(0,welcomeMessage.find_first_of('\r')) << std::endl;
		#endif	
	}
	else if (initiated && !rts) {
		initiated=false;
		#ifdef _GIRAFF_PC_DEBUG_ 
			_printTime(); 
			//std::cout << "Giraff PC  -> Linux PC  : DTR down"<<std::endl;
		#endif	
	}
	return true;
}

inline bool GiraffPC::open()
{
	return SerialInterface::open();
}

inline bool GiraffPC::readCommand(std::string& command)
{
	static char buffer[257];
	command="";	
	if (!initiated) {
		return true;
	}
	int bytes;
	
	if (!SerialInterface::incomingBytes(bytes)) {
		return false;
	}

	if (bytes==0) {
		return true;
	}
	int counter=0;
	do {
		bytes = SerialInterface::read(buffer,256);
		if (bytes==-1) {
			command="";
			return false;
		}
		if (bytes==0) {
			if (counter==100) {			
				break;
			}
			counter++;
			continue;
		}
		else {
			counter=0;
		} 
		buffer[bytes]=0;
		command+=buffer;
	} while(buffer[bytes-1]!='\r');
	
	
	#ifdef _GIRAFF_PC_DEBUG_ 
		if (command.size()>0) {
			_printTime(); 
			//std::cout << "Giraff PC  -> Linux PC  : "<< command.substr(0,command.find_first_of('\r')) << std::endl;
		}
	#endif
	return true;
}

inline bool GiraffPC::writeResponse(const std::string& response)
{
	if (!initiated) {
		return true;
	}
	bool success =  SerialInterface::write(response);
	
	#ifdef _GIRAFF_PC_DEBUG_ 
		_printTime(); 
		//std::cout << "Linux PC   -> Giraff PC : "<< response.substr(0,response.find_first_of('\r')) << std::endl;
	#endif	

	return success;
}

inline bool GiraffPC::isInitiated()
{
	return initiated;
}


//-- END OF INLINE FUNCTIONS ---------------------------------------
#endif
