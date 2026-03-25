#include "BotaFTSensor.h"

#include <cstring>   // memset, strlen
#include <cstdio>    // printf / perror
#include <cmath>     // fabs
#include <thread>    // std::this_thread
#include <chrono>    // std::chrono

// ------------------------------------------------------------------ //
//  Construction / lifecycle                                           //
// ------------------------------------------------------------------ //

BotaFTSensor::BotaFTSensor()
    : _device("")
    , _baudrate(B460800)
    , _fd(-1)
    , _comm(_fd)
    , _tared(false)
{
    memset(_tareOffset, 0, sizeof(_tareOffset));
}

BotaFTSensor::BotaFTSensor(const std::string& device, speed_t baudrate)
    : _device(device)
    , _baudrate(baudrate)
    , _fd(-1)
    , _comm(_fd)
    , _tared(false)
{
    memset(_tareOffset, 0, sizeof(_tareOffset));
}

BotaFTSensor::~BotaFTSensor()
{
    close();
}

void BotaFTSensor::setDevice(const std::string& device, speed_t baudrate)
{
    _device   = device;
    _baudrate = baudrate;
}

// ------------------------------------------------------------------ //
//  Internal helpers                                                   //
// ------------------------------------------------------------------ //

bool BotaFTSensor::applyTermios(int vmin, int vtime)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(_fd, &tty) != 0)
    {
        perror("[BotaFTSensor] tcgetattr failed");
        return false;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN]  = (cc_t)vmin;
    tty.c_cc[VTIME] = (cc_t)vtime;

    cfsetispeed(&tty, _baudrate);
    cfsetospeed(&tty, _baudrate);

    if (tcsetattr(_fd, TCSANOW, &tty) != 0)
    {
        perror("[BotaFTSensor] tcsetattr failed");
        return false;
    }
    return true;
}

bool BotaFTSensor::openPort()
{
    _fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY);
    if (_fd < 0)
    {
        perror(("[BotaFTSensor] Cannot open " + _device).c_str());
        if (errno == EACCES)
            printf("[BotaFTSensor] Hint: add the current user to the 'dialout' group.\n");
        return false;
    }

    if (!applyTermios(1, 0))
    {
        close();
        return false;
    }

    // Enable FTDI low-latency mode
    struct serial_struct ser_info;
    if (ioctl(_fd, TIOCGSERIAL, &ser_info) == 0)
    {
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(_fd, TIOCSSERIAL, &ser_info);
    }

    return true;
}

void BotaFTSensor::softwareReset()
{
    // 'I' triggers a software reset from any state (RUN or CONFIG)
    uint8_t cmd = 'I';
    ::write(_fd, &cmd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    tcflush(_fd, TCIFLUSH);

    // Force RUN mode — after reset the sensor may stay in CONFIG
    cmd = 'R';
    ::write(_fd, &cmd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tcflush(_fd, TCIFLUSH);
}

// ------------------------------------------------------------------ //
//  Lifecycle                                                          //
// ------------------------------------------------------------------ //

bool BotaFTSensor::open()
{
    for (int attempt = 1; attempt <= MAX_OPEN_RETRIES; attempt++)
    {
        printf("[BotaFTSensor] Intento %d/%d...\n", attempt, MAX_OPEN_RETRIES);

        if (!openPort())
            return false;  // hardware error, no point retrying

        if (attempt > 1)
            softwareReset();

        if (!flushStartup())
        {
            close();
            continue;
        }

        if (sanityCheck())
        {
            printf("[BotaFTSensor] Opened %s (fd=%d).\n", _device.c_str(), _fd);
            return true;
        }

        close();
    }

    printf("[BotaFTSensor] ERROR: no se pudo arrancar correctamente tras %d intentos.\n",
           MAX_OPEN_RETRIES);
    return false;
}

void BotaFTSensor::close()
{
    if (_fd >= 0)
    {
        ::close(_fd);
        _fd = -1;
        printf("[BotaFTSensor] Serial port closed.\n");
    }
}

// ------------------------------------------------------------------ //
//  Startup sync — byte-by-byte with VMIN=1                          //
// ------------------------------------------------------------------ //

bool BotaFTSensor::flushStartup()
{
    const int FRAME_SIZE = (int)sizeof(_comm.frame);
    const int MAX_BYTES  = 100000;
    uint8_t   byte       = 0;
    int       bytesRead  = 0;

    while (bytesRead < MAX_BYTES)
    {
        if (::read(_fd, &byte, 1) != 1)
            continue;

        bytesRead++;

        if (byte != 0xAA)
            continue;

        _comm.frame.bytes[0] = 0xAA;
        int got = 1;
        while (got < FRAME_SIZE)
        {
            int n = ::read(_fd, &_comm.frame.bytes[got], 1);
            if (n == 1) got++;
        }

        if (_comm.isCrcOk())
        {
            if (!applyTermios(37, 0))
                return false;

            printf("[BotaFTSensor] Sincronizado, sensor listo.\n");
            return true;
        }
    }

    return false;
}

// ------------------------------------------------------------------ //
//  Sanity check                                                       //
// ------------------------------------------------------------------ //

bool BotaFTSensor::sanityCheck()
{
    int valid = 0;

    for (int i = 0; i < 10; i++)
    {
        while (_comm.serialAvailable() < (int)sizeof(_comm.frame))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (_comm.readFrame() != BotaForceTorqueSensorComm::VALID_FRAME)
            continue;

        const auto& f = _comm.frame.data;

        bool ok = true;
        for (int j = 0; j < 3; j++)
            if (fabs(f.forces[j]) > MAX_FORCE_N)  { ok = false; break; }
        if (ok)
            for (int j = 3; j < 6; j++)
                if (fabs(f.forces[j]) > MAX_TORQUE_NM) { ok = false; break; }

        if (ok) valid++;
    }

    return valid >= 8;
}

// ------------------------------------------------------------------ //
//  Data acquisition — VMIN=37 guarantees one complete frame         //
// ------------------------------------------------------------------ //

BotaFTSensor::ReadResult BotaFTSensor::read(FTData& data)
{
    if (_fd < 0)
        return NO_DATA;

    if (_comm.serialAvailable() < (int)sizeof(_comm.frame))
        return NO_DATA;

    if (::read(_fd, _comm.frame.bytes, sizeof(_comm.frame)) != (int)sizeof(_comm.frame))
        return NO_DATA;

    if (_comm.frame.header != 0xAA)
        return SYNC_ERROR;

    if (!_comm.isCrcOk())
        return CRC_ERROR;

    if (_comm.frame.data.status.val > 0)
        return STATUS_ERROR;

    const auto& f = _comm.frame.data;
    for (int i = 0; i < 6; i++)
        data.forces[i] = f.forces[i];
    data.timestamp   = f.timestamp;
    data.temperature = f.temperature;
    data.status      = {};

    if (_tared)
        for (int i = 0; i < 6; i++)
            data.forces[i] -= _tareOffset[i];

    return VALID;
}

// ------------------------------------------------------------------ //
//  Tare                                                               //
// ------------------------------------------------------------------ //

void BotaFTSensor::tare(int samples)
{
    double accum[6] = {0};
    int count = 0;

    while (count < samples)
    {
        while (_comm.serialAvailable() < (int)sizeof(_comm.frame))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        switch (_comm.readFrame())
        {
            case BotaForceTorqueSensorComm::VALID_FRAME:
                if (_comm.frame.data.status.val == 0)
                {
                    for (int i = 0; i < 6; i++)
                        accum[i] += _comm.frame.data.forces[i];
                    count++;
                }
                break;
            default:
                break;
        }
    }

    for (int i = 0; i < 6; i++)
        _tareOffset[i] = (float)(accum[i] / samples);

    _tared = true;
    printf("[BotaFTSensor] Tare completado. Offsets: Fx=%.4f Fy=%.4f Fz=%.4f Tx=%.4f Ty=%.4f Tz=%.4f\n",
        _tareOffset[0], _tareOffset[1], _tareOffset[2],
        _tareOffset[3], _tareOffset[4], _tareOffset[5]);
}

void BotaFTSensor::resetTare()
{
    _tared = false;
    memset(_tareOffset, 0, sizeof(_tareOffset));
    printf("[BotaFTSensor] Tare reseteado.\n");
}

uint32_t BotaFTSensor::getCrcErrorCount()
{
    return _comm.get_crc_count();
}
