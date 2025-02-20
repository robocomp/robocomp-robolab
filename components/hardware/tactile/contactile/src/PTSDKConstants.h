#ifndef PTSDKCONTANTS_H
#define PTSDKCONTANTS_H

/* Input and output parameters */

#define IN					// Indicates that a parameter is an input parameter.
#define OUT					// Indicates that a parameter is an output parameter.

/* Constants related to the data packet */

#define STARTBYTE0 		0x55		// The first byte of the start packet
#define STARTBYTE1 		0x66		// The second byte of the start packet
#define STARTBYTE2 		0x77		// The third byte of the start packet
#define STARTBYTE3 		0x88		// The fourth byte of the start packet
#define ENDBYTE0		0xAA		// The first byte of the end packet
#define ENDBYTE1		0xBB		// The second byte of the end packet
#define ENDBYTE2		0xCC		// The third byte of the end packet
#define ENDBYTE3		0xDD		// The fourth byte of the end packet


/* Constants related to dimensions*/

#define X_IND			0		// The index of the X-dimension.
#define Y_IND			1		// The index of the Y-dimension.
#define Z_IND			2		// The index of the Z-dimension.
#define NDIM			3		// The number of dimensions

/* Constants related to sensors */

#define MAX_NSENSOR		20		// The maximum number of sensors connected to the communication hub
#define MAX_NPILLAR		100		// The maximum number of pillars in a sensor
#define MAX_NTEMPSENSOR		4		// The maximum number of on-board temperature sensors

/* Constants related to log file */

#define LOG_RATE_100		100		// Constant representing 100 Hz log file rate
#define LOG_RATE_500		500		// Constant representing 500 Hz log file rate
#define LOG_RATE_1000		1000		// Constant representing 1000 Hz log file rate

/* ----------------------------- */

#define ISDEBUGOUTPUT		0

#define LOG_BUF_LEN		5000		// Length of the log file data buffer - the maximum number of bytes in a single line of the log file
#define LOG_FILENAME_LEN	200		// Length of the log filename buffer - the maximum number of characters in the log filename

#define DISPLACEMENT_IND	0		// Displacement measurement - used to unpack calibration file
#define FORCE_IND		1		// Force measurement - used to unpack calibration file
#define NMEASUREMENT		2		// Number of measurements - used to unpack calibration file

#endif
