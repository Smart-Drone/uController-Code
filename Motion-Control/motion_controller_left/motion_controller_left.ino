// Include directives for MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Include directive for Wi-Fi transmission
#include "UDPServer.h"

#include "OffsetStore.h"

//==============================================================================================================
//=                                           GENERAL VARIABLES                                                =
//==============================================================================================================
constexpr int BAUD_RATE = 115200;
constexpr int CAL_BTN_PIN = D6; // Pin for calibration button

OffsetStore store;

//==============================================================================================================
//=                                           MPU6050 VARIABLES                                                =
//==============================================================================================================
MPU6050 mpu; // Class default I2C address is 0x68
constexpr int INTERRUPT_PIN = D7;

// Control/status variables
bool dmp_ready = false;              //  Set true if DMP init was successful
volatile bool mpu_interrupt = false; // Indicates whether MPU interrupt pin has gone high
uint8_t mpu_int_status;              // Holds actual interrupt status byte from MPU
uint8_t dev_status;                  // Return status after each device operation (0 = success, !0 = error)
uint16_t packet_size;                // Expected DMP packet size (default is 42 bytes)
uint16_t fifo_count;                 // Count of all bytes currently in FIFO
uint8_t fifo_buffer[64];             // FIFO storage buffer

// Orientation/motion variables
Quaternion q;         // [w, x, y, z]         Quaternion container
int16_t accel_raw[3]; // [x, y, z]            Accel sensor measurements
int16_t gyro_raw[3];  // [x, y, z]            Gyro sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Calibration variables
constexpr int cal_buffer_size = 1000;      // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
constexpr int accel_deadzone = 8;          // Accelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
constexpr int gyro_deadzone = 1;           // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t accel_offset[3];                   // [x, y, z]   Accel offsets
int16_t gyro_offset[3];                    // [x, y, z]   Gyro offsets
int16_t accel_mean[3];                     // [x, y, z]   Accel average readings
int16_t gyro_mean[3];                      // [x, y, z]   Gyro average readings
constexpr int ACCEL_SCALE_FACTOR = 16384;  // Factor for converting raw accel values to gravity values (only needed during calibration)


//================================================================================================================
//=                                             WIFI VARIABLES                                                   =
//================================================================================================================
UDPServer* udp_server;                                // Provides interface for Wi-Fi communication
constexpr unsigned int LOCAL_UDP_PORT = 4210;         // Port number by which the device can be found
constexpr char* WIFI_SSID = "PARROT_NEST";            // The device will try to connect to this network
constexpr char* WIFI_PWD = "pollycracker";            // This password will be used for connecting to the network
bool wifi_ready = false;                              // Set true if Wi-Fi setup was successful


//================================================================================================================
//=                                             INITIAL SETUP                                                    =
//================================================================================================================
void setup() {
    /******************
    *  GENERAL SETUP  *
    ******************/
    // Initialize serial communication
    Serial.begin(BAUD_RATE);

    // Set pin for calibration button
    pinMode(CAL_BTN_PIN, INPUT);


    /***************
    *  WIFI SETUP  *
    ***************/
    // Start message
    Serial.println("WI-FI SETUP");

    // Print MAC Address
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Create interface for Wi-Fi transmission
    udp_server = new UDPServer(WIFI_SSID, WIFI_PWD, LOCAL_UDP_PORT);

    // Try to establish Wi-Fi connection
    Serial.print("Trying to connect to network ");
    Serial.println(WIFI_SSID);
    if(udp_server->initialize()) {
        Serial.println("Connection established!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        wifi_ready = true;
    }
    else {
        Serial.println("CONNECTION FAILED!");
    }


    /******************
    *  MPU6050 SETUP  *
    ******************/
    // Start message
    Serial.println("MPU6050 SETUP");

    // Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    // Initialize device
    Serial.println("Initializing device...");
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    Serial.println("Testing device connection...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    dev_status = mpu.dmpInitialize();

    // Supply estimated offsets (calibration is necessary)
    store.loadOffsets(accel_offset, gyro_offset);
    Serial.print("Loaded Offsets: ");
    Serial.print(accel_offset[0]); Serial.print(" ");
    Serial.print(accel_offset[1]); Serial.print(" ");
    Serial.print(accel_offset[2]); Serial.print(" ");
    Serial.print(gyro_offset[0]); Serial.print(" ");
    Serial.print(gyro_offset[1]); Serial.print(" ");
    Serial.print(gyro_offset[2]); Serial.println(" ");
    mpu.setXAccelOffset(accel_offset[0]);
    mpu.setYAccelOffset(accel_offset[1]);
    mpu.setZAccelOffset(accel_offset[2]);
    mpu.setXGyroOffset(gyro_offset[0]);
    mpu.setYGyroOffset(gyro_offset[1]);
    mpu.setZGyroOffset(gyro_offset[2]);

    if (dev_status == 0) { // Make sure DMP initalization was successful
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.println("Enabling interrupt detection...");
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpu_int_status = mpu.getIntStatus();

        // Set DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println("DMP ready! Waiting for first interrupt...");
        dmp_ready = true;

        // Get expected DMP packet size for later comparison
        packet_size = mpu.dmpGetFIFOPacketSize();
    }
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print("DMP Initialization failed (code ");
        Serial.print(dev_status);
        Serial.println(")");
    }
}


//================================================================================================================
//                                            MAIN PROGRAM LOOP                                                  =
//================================================================================================================
void loop() {
    // If setup failed, don't try to do anything
    if (!dmp_ready || !wifi_ready) return;


    /************************
    *  MPU6050 CALIBRATION  *
    ************************/
    // Start calibration routine when user presses calibration button
    int btn_state = digitalRead(CAL_BTN_PIN);
    if(btn_state == HIGH) {
        calibrateMPU6050();
    }


    /****************
    *  DMP READING  *
    ****************/
    // Wait for MPU interrupt or extra packet(s) available
    while (!mpu_interrupt && fifo_count < packet_size);

    // Reset interrupt flag and get INT_STATUS byte
    mpu_interrupt = false;
    mpu_int_status = mpu.getIntStatus();

    // Get current FIFO count
    fifo_count = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless code is too inefficient)
    if ((mpu_int_status & 0x10) || fifo_count == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpu_int_status & 0x02) {
        // Wait for correct available data length, should be a VERY short wait
        while (fifo_count < packet_size) fifo_count = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifo_buffer, packet_size);

        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifo_count -= packet_size;

        // Calculate pitch/roll and print angles in degrees
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("PITCH/ROLL:\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    }


    /***********************
    *  WI-FI TRANSMISSION  *
    ***********************/
    // If no request was received yet check for request
    if(udp_server->getState() == UDPServer::WAITING_FOR_REQUEST) {
        Serial.println("Waiting for request...");
        udp_server->listen();
    }
    // If request was received transmit pitch/roll data
    if(udp_server->getState() == UDPServer::SENDING) {
        //char packet[3] = {0,(char) (ypr[1]*180/ M_PI), (char) (ypr[2]*180/M_PI)};
        Serial.println("Sending data...");
        char pitch = ypr[1] * 180/M_PI;
        char roll = ypr[2] * 180/M_PI;
        char packet[3] = {2, pitch, roll};
        //char packet[3] = {-6, -6, -6};
        udp_server->sendPacket(packet, 3);
    }
}


//================================================================================================================
//                                       INTERRUPT DETECTION ROUTINE                                             =
//================================================================================================================
void dmpDataReady() {
    mpu_interrupt = true;
}


//================================================================================================================
//                                           CALIBRATION ROUTINE                                                 =
//================================================================================================================
void calibrateMPU6050() {
    // Start message
    Serial.println("Starting MPU6050 sensor calibration...");

    // Reset offsets
    Serial.println("Resetting offsets...");
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // Average sensor readings
    meanSensors();

    // Estimate offsets
    accel_offset[0] = -accel_mean[0] / 8;
    accel_offset[1] = -accel_mean[1] / 8;
    accel_offset[2] = (ACCEL_SCALE_FACTOR - accel_mean[2]) / 8; // Accel scale factor = 16384
    gyro_offset[0] = -gyro_mean[0] / 4;
    gyro_offset[1] = -gyro_mean[1] / 4;
    gyro_offset[2] = -gyro_mean[2] / 4;

    // Calibration loop
    int ready;
    while(ready != 6) {
        // Info message
        Serial.println("Calibrating...");

        ready = 0;

        // Set offsets
        mpu.setXAccelOffset(accel_offset[0]);
        mpu.setYAccelOffset(accel_offset[1]);
        mpu.setZAccelOffset(accel_offset[2]);
        mpu.setXGyroOffset(gyro_offset[0]);
        mpu.setYGyroOffset(gyro_offset[1]);
        mpu.setZGyroOffset(gyro_offset[2]);

        // Average sensor readings
        meanSensors();

        // Comperaing average readings to fault tolerance
        if (abs(accel_mean[0]) <= accel_deadzone) ready++;
        else accel_offset[0] -= accel_mean[0] / accel_deadzone;
        if (abs(accel_mean[1]) <= accel_deadzone) ready++;
        else accel_offset[1] -= accel_mean[1] / accel_deadzone;
        if (abs(ACCEL_SCALE_FACTOR - accel_mean[2]) <= accel_deadzone) ready++;
        else accel_offset[2] += (ACCEL_SCALE_FACTOR - accel_mean[2]) / accel_deadzone;
        if (abs(gyro_mean[0]) <= gyro_deadzone) ready++;
        else gyro_offset[0] -= gyro_mean[0] / (gyro_deadzone + 1);
        if (abs(gyro_mean[1]) <= gyro_deadzone) ready++;
        else gyro_offset[1] -= gyro_mean[1] / (gyro_deadzone + 1);
        if (abs(gyro_mean[2]) <= gyro_deadzone) ready++;
        else gyro_offset[2] -= gyro_mean[2] / (gyro_deadzone + 1);
    }

    store.saveOffsets(accel_offset, gyro_offset);

    // Completion message
    Serial.println("Calibration completed");
    Serial.println("Average readings with offsets:");
    Serial.print("AX/AY/AZ/GX/GY/GZ:\t");
    Serial.print(accel_mean[0]); Serial.print("\t");
    Serial.print(accel_mean[1]); Serial.print("\t");
    Serial.print(accel_mean[2]); Serial.print("\t");
    Serial.print(gyro_mean[0]); Serial.print("\t");
    Serial.print(gyro_mean[1]); Serial.print("\t");
    Serial.println(gyro_mean[2]);
    Serial.println("Chosen offsets:\t");
    Serial.print(accel_offset[0]); Serial.print("\t");
    Serial.print(accel_offset[1]); Serial.print("\t");
    Serial.print(accel_offset[2]); Serial.print("\t");
    Serial.print(gyro_offset[0]); Serial.print("\t");
    Serial.print(gyro_offset[1]); Serial.print("\t");
    Serial.println(gyro_offset[2]);
}


//================================================================================================================
//                                      CALIBRATION AVERAGING ROUTINE                                            =
//================================================================================================================
void meanSensors() {
    int accel_buffer[] = {0, 0, 0}; // [x, y, z]   Accel buffers
    int gyro_buffer[] = {0, 0, 0};  // [x, y, z]   Gyro buffers

    // Averaging loop
    for (int i = 0; i < cal_buffer_size + 101; ++i) { // First 100 measurements are discarded
        // Read raw accel/gyro measurements from device
        mpu.getMotion6(&accel_raw[0], &accel_raw[1], &accel_raw[2], &gyro_raw[0],  &gyro_raw[1],  &gyro_raw[2]);

        if (i > 100 && i <= cal_buffer_size + 100){
            // Calculating sums
            accel_buffer[0] += accel_raw[0];
            accel_buffer[1] += accel_raw[1];
            accel_buffer[2] += accel_raw[2];
            gyro_buffer[0] += gyro_raw[0];
            gyro_buffer[1] += gyro_raw[1];
            gyro_buffer[2] += gyro_raw[2];
        }

        if (i == cal_buffer_size + 100){
            // Averaging measurements
            accel_mean[0] = accel_buffer[0] / cal_buffer_size;
            accel_mean[1] = accel_buffer[1] / cal_buffer_size;
            accel_mean[2] = accel_buffer[2] / cal_buffer_size;
            gyro_mean[0] = gyro_mean[0] / cal_buffer_size;
            gyro_mean[1] = gyro_mean[1] / cal_buffer_size;
            gyro_mean[2] = gyro_mean[2] / cal_buffer_size;
        }

        // Wait to avoid repeated meassures
        delay(2);
    }
}
