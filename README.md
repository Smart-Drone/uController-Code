# uController-Code

## Dependencies

### Build environment:
Arduino IDE 1.8.6  
ESP8266 Arduino Core ([Install instructions](https://arduino-esp8266.readthedocs.io/en/latest/installing.html))

### Hardware:
GY-521/MPU6050  
NodeMCU DEVKIT 1.0

### Libraries:
[jrowberg/i2cdevlib/Arduino/MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)  
[jrowberg/i2cdevlib/Arduino/I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev)

## Library adaptation:
Replace
```
#include <avr/pgmspace.h>
```
with
```
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif
```
in the following files: 
* MPU6050.h
* MPU6050_6Axis_MotionApps20.h
* MPU6050_9Axis_MotionApps41.h

This change is necessary to port the MPU6050 Arduino library to ESP8266.
