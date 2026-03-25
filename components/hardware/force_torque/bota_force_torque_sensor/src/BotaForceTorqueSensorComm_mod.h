#include <stdio.h>
#include <stdint.h>

class BotaForceTorqueSensorComm
{
private:
  bool _synced;
  uint32_t _crc_err_count;
  union DataStatus
  {
      struct __attribute__((__packed__))
      {
          uint16_t app_took_too_long:1;
          uint16_t overrange:1;
          uint16_t invalid_measurements:1;
          uint16_t raw_measurements:1;
          uint16_t:12; //reserved
      };
      uint16_t val;
      uint8_t bytes[1];
  };
  union AppOutput
  {
      struct __attribute__((__packed__))
      {
          DataStatus status;
          float forces[6];
          uint32_t timestamp;
          float temperature;
      };
      uint8_t bytes[1];
  };
  union RxFrame
  {
      struct __attribute__((__packed__))
      {
          uint8_t header;
          AppOutput data;
          uint16_t crc;
      };
      uint8_t bytes[1];
  };

public:
  enum ReadFrameRes {NO_FRAME, VALID_FRAME, NOT_VALID_FRAME, NOT_ALLIGNED_FRAME};
  BotaForceTorqueSensorComm();
  uint16_t crc16_x25(uint8_t* data, size_t len);
  uint16_t crc16_ccitt_false(uint8_t* data, size_t len);
  uint16_t crc16_mcrf4xx(uint8_t *data, size_t len);
  bool isCrcOk();
  bool checkSync();
  bool isSynced() {return _synced;}
  uint32_t get_crc_count() {return _crc_err_count;}
  virtual int serialAvailable() = 0;
  virtual int serialReadBytes(uint8_t* data, size_t len) = 0;
  ReadFrameRes readFrame();
  RxFrame frame;
};
