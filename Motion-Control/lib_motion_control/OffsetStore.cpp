#include "OffsetStore.h"

OffsetStore::OffsetStore() {

}

void OffsetStore::saveOffsets(int16_t* accel_offset, int16_t* gyro_offset){
    EEPROM.begin(EEPROM_SIZE);

    EEPROM.put(ADDRESS_AX,accel_offset[0]);
    EEPROM.put(ADDRESS_AY,accel_offset[1]);
    EEPROM.put(ADDRESS_AZ,accel_offset[2]);
    EEPROM.put(ADDRESS_GX,gyro_offset[0]);
    EEPROM.put(ADDRESS_GY,gyro_offset[1]);
    EEPROM.put(ADDRESS_GZ,gyro_offset[2]);

    EEPROM.put(CONTROL_ADDRESS, 0x01);

    EEPROM.commit();
}

void OffsetStore::loadOffsets(int16_t* accel_offset, int16_t* gyro_offset){
    EEPROM.begin(EEPROM_SIZE);
    
    byte control_val;
    EEPROM.get(CONTROL_ADDRESS, control_val);

    if(control_val == 1) {
        EEPROM.get(ADDRESS_AX,accel_offset[0]);
        EEPROM.get(ADDRESS_AY,accel_offset[1]);
        EEPROM.get(ADDRESS_AZ,accel_offset[2]);
        EEPROM.get(ADDRESS_GX,gyro_offset[0]);
        EEPROM.get(ADDRESS_GY,gyro_offset[1]);
        EEPROM.get(ADDRESS_GZ,gyro_offset[2]);
    }
    else {
        accel_offset[0] = 0;
        accel_offset[1] = 0;
        accel_offset[2] = 0;
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;
    }
}
