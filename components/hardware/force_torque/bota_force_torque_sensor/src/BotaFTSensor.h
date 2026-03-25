#pragma once

#include <string>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "BotaForceTorqueSensorComm.h"

/**
 * @brief Wrapper around BotaForceTorqueSensorComm that manages the serial
 *        port lifecycle and exposes a simple polling API for use inside a
 *        RoboComp component.
 *
 * Typical usage inside a RoboComp component:
 *
 *   BotaFTSensor sensor("/dev/bota_ft_sensor");
 *   if (!sensor.open()) { ... handle error ... }
 *   sensor.tare(100);
 *
 *   BotaFTSensor::FTData data;
 *   switch (sensor.read(data))
 *   {
 *     case BotaFTSensor::VALID:        break;
 *     case BotaFTSensor::STATUS_ERROR: break;
 *     case BotaFTSensor::CRC_ERROR:    break;
 *     case BotaFTSensor::SYNC_ERROR:   break;
 *     case BotaFTSensor::NO_DATA:      break;
 *   }
 *
 *   sensor.close();
 */
class BotaFTSensor
{
public:
    // ------------------------------------------------------------------ //
    //  Public data types                                                  //
    // ------------------------------------------------------------------ //

    struct FTData
    {
        float    forces[6];   ///< Fx Fy Fz Tx Ty Tz  [N / Nm]
        uint32_t timestamp;   ///< Sensor-internal timestamp [us]
        float    temperature; ///< Board temperature [°C]

        struct Status
        {
            bool app_took_too_long;
            bool overrange;
            bool invalid_measurements;
            bool raw_measurements;
        } status;
    };

    enum ReadResult
    {
        VALID,
        STATUS_ERROR,
        CRC_ERROR,
        SYNC_ERROR,
        NO_DATA
    };

    // ------------------------------------------------------------------ //
    //  Construction / lifecycle                                           //
    // ------------------------------------------------------------------ //

    BotaFTSensor();
    explicit BotaFTSensor(const std::string& device, speed_t baudrate = B460800);
    ~BotaFTSensor();

    void setDevice(const std::string& device, speed_t baudrate = B460800);

    /**
     * @brief Open the serial port, synchronise to the data stream and verify
     *        that the sensor is streaming calibrated data. Retries up to
     *        MAX_OPEN_RETRIES times, issuing a software reset between attempts.
     * @return true on success, false if the sensor could not be initialised.
     */
    bool open();

    void close();

    bool isOpen() const { return _fd >= 0; }

    // ------------------------------------------------------------------ //
    //  Data acquisition                                                   //
    // ------------------------------------------------------------------ //

    ReadResult read(FTData& data);

    uint32_t getCrcErrorCount();
    int      getAvailable() { return _comm.serialAvailable(); }

    // ------------------------------------------------------------------ //
    //  Tare / zero offset                                                 //
    // ------------------------------------------------------------------ //

    void tare(int samples = 100);
    void resetTare();
    bool isTared() const { return _tared; }

private:
    // ------------------------------------------------------------------ //
    //  Inner concrete subclass                                            //
    // ------------------------------------------------------------------ //
    class Impl : public BotaForceTorqueSensorComm
    {
    public:
        explicit Impl(int& fd) : _fd(fd) {}

        int serialReadBytes(uint8_t* data, size_t len) override
        {
            return ::read(_fd, data, len);
        }

        int serialAvailable() override
        {
            int bytes = 0;
            ioctl(_fd, FIONREAD, &bytes);
            return bytes;
        }

    private:
        int& _fd;
    };

    // ------------------------------------------------------------------ //
    //  Private methods                                                    //
    // ------------------------------------------------------------------ //

    bool applyTermios(int vmin, int vtime);
    bool openPort();
    void softwareReset();
    bool flushStartup();
    bool sanityCheck();

    // ------------------------------------------------------------------ //
    //  Private members                                                    //
    // ------------------------------------------------------------------ //
    std::string _device;
    speed_t     _baudrate;
    int         _fd;
    Impl        _comm;

    bool  _tared;
    float _tareOffset[6];

    static constexpr int   MAX_OPEN_RETRIES = 5;
    static constexpr float MAX_FORCE_N      = 500.0f;
    static constexpr float MAX_TORQUE_NM    = 50.0f;
};
