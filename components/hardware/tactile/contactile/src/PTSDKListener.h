#ifndef PTSDKLISTENER_H
#define PTSDKLISTENER_H
#endif

#ifdef _WIN32
#ifndef PTSDK_CPP_LIB_H
#include "PTSDK_CPP_LIB.h"
#endif
#endif

#ifndef PTSDKCONSTANTS_H
#include "PTSDKConstants.h"
#endif

#ifndef PTSDKSENSOR_H
#include "PTSDKSensor.h"
#endif

#include <stdio.h>
#include <stdint.h>
//#include <string>

#ifdef _WIN32
#include <windows.h>
#endif
#ifndef _WIN32
#include <cstddef>
#include <cstdint>
#include <stdint.h>
#include <pthread.h>
#define BYTE byte
#define DWORD uint32_t
#define HANDLE int
#endif

#define NBYTES_STARTPACKET		4		// Number of bytes in the start packet
#define NBYTES_ENDPACKET		4		// Number of bytes in the end packet
#define MAX_DATAPACKETLEN		1024		// Maximum number of bytes in the data packet (the actual number of bytes may be variable)

using namespace std;

#ifdef _WIN32
void PTSDK_CPP_LIB_API PTSDKListener_event_thread_start(void* arg);
#else
void* PTSDKListener_event_thread_start(void* arg);
#endif

/** \brief The PTSDKListener class describes a listener for the Communications Hub with a number of PapillArray Tactile Sensors connected.
 *
 * The PTSDKListener class describes a listener for the Communications Hub with a number of PapillArray Tactile Sensors connected.
 *
 * @author Contactile Pty Ltd
 * @version March 2020
 */
#ifdef _WIN32
class PTSDK_CPP_LIB_API PTSDKListener {
#else
class PTSDKListener {
#endif

private:
	bool isLogging;
	bool isWaitingForStartByte;				// True if PTSDKListener is waiting for the next start byte; false otherwise
	int waitingForEndByteNum;
	BYTE startBytes[NBYTES_STARTPACKET];			// The bytes to match to a start byte from the communications hub
	BYTE endBytes[NBYTES_ENDPACKET];

	BYTE startPacketBuf[NBYTES_STARTPACKET];
	BYTE dataPacketBuf[MAX_DATAPACKETLEN];			// A buffer containing the data packet read from the COM port

	int nSensor;						// The number of sensors connected to the communications hub
	PTSDKSensor* pSensors[MAX_NSENSOR];			// Sensors connected to the communications hub

#ifdef _WIN32
	HANDLE serial_handle;					// The handle to the serial port
#else
	int serial_handle;
#endif
#ifdef _WIN32
	HANDLE thread_handle;					// The handle to the thread for listening to the COM port
#else
	pthread_t thread_handle;
#endif
	unsigned int  threadid;					// The id of the thread for listening to the COM port
	bool isListening;					// Set to true when the thread begins, and set to false when we want to kill the thread

	HANDLE logFile_handle;					// The handle to the log file
	char logBuf[LOG_BUF_LEN];				// Buffer for the log file
	int samplesToSkip;					// Number of samples to skip for logging
	int sampleCounter;					// The number of samples read from the comms hub
	bool isLogThisSample;					// Whether or not to write the current sample to the log file

	bool readByte(OUT BYTE* pVal);
	bool readUint8(OUT uint8_t* pVal);
	bool readUint16(OUT uint16_t* pVal);
	bool readUint32(OUT uint32_t* pVal);

	bool parseDataPacket(BYTE* dataPacketBuf, uint32_t nDataPacketBuf);

	bool validateChecksum(IN const BYTE* dataPacketBuf,
		IN const uint32_t nDataPacketBuf,
		IN uint32_t checksum_index);

	void unpackAddress(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		IN uint16_t addressSize,
		IN OUT uint32_t* pVal);
	void unpackUint8(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT uint8_t* pVal);
	void unpackUint16(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT uint16_t* pVal);
	void unpackUint32(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT uint32_t* pVal);
	void unpackUint64(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT uint64_t* pVal);

	void unpackFloat(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT float* pVal);

	void unpackDouble(IN const BYTE data[MAX_DATAPACKETLEN],
		IN OUT uint32_t* pByteInd,
		OUT double* pVal);

	void startListening(void);
	void stopListening(void);

	bool openLogFile(void);
	void closeLogFile(void);
	bool writeHeaderToLog(void);
	bool writeSampleToLog(void);

#ifndef _WIN32
	bool ReadFile(int file_handle, BYTE* buf, DWORD nBytesToRead, DWORD* nBytesRead, void* pOverlapped);
	bool WriteFile(int file_handle, char* buf, DWORD nBytesToWrite, DWORD* nBytesWritten, void* pOverlapped);
	int strcat_s(char* dest, int destsz, const char* src);
	void Sleep(int milliseconds);
#endif

public:

	/* Constructors */

	/**
	 * Default constructor.
	 */
	PTSDKListener(IN const bool logFlag);

	/* Destructors */

	/**
	 * Destructor.
	 */
	~PTSDKListener();

	/* Public member methods */

	/**
	 * Adds a sensor object to the PTSDKListener.
	 *
	 * @param[in] pSensor A pointer to the sensor object.
	 */
	void addSensor(IN PTSDKSensor* pSensor);

	/**
	 * Connects to the COM port
	 *
	 * @param[in] port The COM port name.
	 * @param[in] rate The rate of the connection.
	 * @param[in] parity The parity of the connection.
	 * @param[in] byteSize The byte size for the connection.
	 *
	 * @return 0 if successfully connected, error code if unsuccessful.
	 */
	int connect(IN const char* portArg,
		IN int const baudRateArg,
		IN const int parityArg,
		IN const char byteSizeArg);

	/**
	 * Connects to the COM port and starts listening for data (starts the listening thread),
	 * processes the data and logs the data to a log file.
	 *
	 * @param[in] port The COM port name.
	 * @param[in] rate The rate of the connection.
	 * @param[in] parity The parity of the connection.
	 * @param[in] byteSize The byte size for the connection.
	 * @param[in] logFileRate The sampling rate for the log file: LOG_RATE_100, LOG_RATE_500 or LOG_RATE_1000
	 *
	 * @return 0 if successfully connected, error code if unsuccessful.
	 */
	int connectAndStartListening(IN const char* port,
		IN const int rate,
		IN const int parity,
		IN const char byteSize,
		IN const int logFileRate);

	/**
	 * The 'infinite' loop of the listening thread.
	 * The thread implementation necessitates that this is a public member function.
	 * However, this function should never be called except through the startListening function
	 * when the listening thread is spawned.
	 */
	void run(void);

	/**
	 * Reads the next data sample from the COM port and adds the data to the Sensor objects associated with this Listener
	 *
	 * @return true if successfully read the data, false if unsuccessful.
	 */
	bool readNextSample(void);

	/**
	 * Stops listening for data from the serial port (and kills the listening thread), stops logging data to the log file
	 * and disconnects from the COM port.
	 */
	void stopListeningAndDisconnect(void);

	/**
	 * Disconnects from the COM port.
	 */
	void disconnect(void);


	/**
	 * Sends a bias request to the communications hub.
	 * A bias should be performed after connecting to the serial port and starting to stream data with the sensor unloaded.
	 * A bias should be performed each time the sensor is known to be unloaded.
	 * A bias operation takes approximately 2 s. Ensure that the sensor remains unloaded throughout this time.
	 * @return true if successfully sent the request, false if unsuccessful.
	 */
	bool sendBiasRequest(void);

};
