#ifndef CONSTANTS_H
#define CONSTANTS_H



#define LASER_DRIVER_PROPERTY_NAME        "Laser.Driver"
#define LASER_DRIVER_PROPERTY_DEFAULT     "HokuyoURG"
#define LASER_DEVICE_PROPERTY_NAME        "Laser.Device"
#define LASER_DEVICE_PROPERTY_DEFAULT     "/dev/ttyACM0"
#define LASER_START_PROPERTY_NAME         "Laser.StartValue"
#define LASER_START_PROPERTY_DEFAULT      "0"
#define LASER_END_PROPERTY_NAME           "Laser.EndValue"
#define LASER_END_PROPERTY_DEFAULT        "768"
#define LASER_SKIP_PROPERTY_NAME          "Laser.SkipValue"
#define LASER_SKIP_PROPERTY_DEFAULT       "1"
#define LASER_SAMPLERATE_PROPERTY_NAME    "Laser.SampleRate"
#define LASER_SAMPLERATE_PROPERTY_DEFAULT "100"

#define LASER_RESOLUTION_PROPERTY_NAME    "Laser.angleRes"
#define LASER_ANGLE_RESOLUTION_DEFAULT     "0.00613593"			// 0.3515625*M_PI/180
#define LASER_INITIAL_ANGLE_PROPERTY_NAME "Laser.angleIni"
#define LASER_INITIAL_ANGLE_DEFAULT 	  "-2.356197"			//-135*M_PI/180

#define LASER_CLUSTER_PROPERTY_DEFAULT 	  "1"		
#define LASER_CLUSTER_PROPERTY_NAME 	  "Laser.Cluster"

#define LASER_MAX_RANGE_PROPERTY_NAME     "Laser.maxRange"
#define LASER_MAX_RANGE_DEFAULT 	  "4094"
#define LASER_MIN_RANGE_PROPERTY_NAME     "Laser.minRange"
#define LASER_MIN_RANGE_DEFAULT			"40"
#define LASER_STEP_MIN_ANGLE_DEFAULT 	"44"

#define LASER_TALKTOBASE_PROPERTY_NAME	"Laser.TalkToBase"
#define LASER_TALKTOBASE_DEFAULT 		"false"

#define LASER_MAX_DEGREES_PROPERTY_NAME	 "Laser.maxDegrees"
#define LASER_MAX_DEGREES_DEFAULT	  	 "240"
#define LASER_STATIC_CONF_PROPERTY_NAME	  "Laser.staticConf"
#define LASER_STATIC_CONF_DEFAULT	  	 "1"

// Laser poweroff, in mseconds
#define LASER_TIMEOUT   10000

// Laser commands sizes
#define LASER_CMD_GET_DATA_SZ   10
#define LASER_CMD_POWER_SZ      3

// Laser commands
#define LASER_CMD_POWER_ON  "L1\n"
#define LASER_CMD_POWER_OFF "L0\n"

#endif
