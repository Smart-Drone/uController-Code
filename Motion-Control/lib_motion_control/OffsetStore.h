#ifndef OFFSET_STORE_H
#define OFFSET_STORE_H

#include <Arduino.h>
#include <EEPROM.h>

class OffsetStore {
public:
  OffsetStore();
  void saveOffsets(int16_t* accel_offset, int16_t* gyro_offset);
  void loadOffsets(int16_t* accel_offset, int16_t* gyro_offset);

private:
  static constexpr int ADDRESS_AX = 0;
  static constexpr int ADDRESS_AY = 2;
  static constexpr int ADDRESS_AZ = 4;
  static constexpr int ADDRESS_GX = 6;
  static constexpr int ADDRESS_GY = 8;
  static constexpr int ADDRESS_GZ = 10;
  static constexpr int CONTROL_ADDRESS = 12; //set to 1 when offsets have been stored
  static constexpr int EEPROM_SIZE = 13;
};

#endif
