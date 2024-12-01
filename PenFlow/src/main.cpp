#include <Wire.h>
#include "MPU9250.h"
#define SerialDebug true   // set to true to get Serial output for debugging

void myinthandler();
bool initMPU();


/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
float   magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer
float SelfTest[6];    // holds results of gyro and accelerometer self test

int  intPin = 0;  //  MPU9250 interrupt
MPU9250 mpu(intPin); // instantiate MPU9250 class

bool intFlag = false;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22 for ESP32
    delay(1000);

    // Initialize MPU9250
    mpu.I2Cscan();
    if (!initMPU()){
        Serial.println("Configuration failed");
        while(1);
    }
    
    delay(1000);

    // Initialize Magnetometer (AK8963)
    float magCal[3];
    mpu.initAK8963(MFS_16BITS, M_100Hz, magCal); // 16-bit magnetometer, 100Hz sampling
    Serial.println("Magnetometer initialized.");
}

void loop() {
    // Rohdaten abrufen
    int16_t accelData[3];
    int16_t gyroData[3];
    int16_t magData[3];

    mpu.readAccelData(accelData);
    mpu.readGyroData(gyroData);
    mpu.readMagData(magData);

    // Daten senden
    Serial.print("{");
    Serial.print("\"accel\": [");
    Serial.print(accelData[0]); Serial.print(", ");
    Serial.print(accelData[1]); Serial.print(", ");
    Serial.print(accelData[2]); Serial.print("], ");

    Serial.print("\"gyro\": [");
    Serial.print(gyroData[0]); Serial.print(", ");
    Serial.print(gyroData[1]); Serial.print(", ");
    Serial.print(gyroData[2]); Serial.print("], ");

    Serial.print("\"mag\": [");
    Serial.print(magData[0]); Serial.print(", ");
    Serial.print(magData[1]); Serial.print(", ");
    Serial.print(magData[2]); Serial.println("]}");
    delay(10); // 100 Hz Sampling
}

bool initMPU(){
    Serial.println("MPU9250 9-axis motion sensor...");
    uint8_t c = mpu.getMPU9250ID();
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
    delay(1000);

    if(c== 0x71){
        Serial.println("MPU9250 is online...");
        
        mpu.resetMPU9250(); // start by resetting MPU9250
        
        mpu.SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
        delay(1000);

        // get sensor resolutions, only need to do this once
        aRes = mpu.getAres(Ascale);
        gRes = mpu.getGres(Gscale);
        mRes = mpu.getMres(Mscale);

        // Comment out if using pre-measured, pre-stored offset biases
        mpu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
        Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
        delay(1000); 
        
        mpu.initMPU9250(Ascale, Gscale, sampleRate); 
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = mpu.getAK8963CID();  // Read WHO_AM_I register for AK8963
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
        delay(1000); 
        
        // Get magnetometer calibration from AK8963 ROM
        mpu.initAK8963Slave(Mscale, Mmode, magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        // Comment out if using pre-measured, pre-stored offset biases
        //  MPU9250.magcalMPU9250(magBias, magScale);
        Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
        Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
        delay(2000); // add delay to see results before serial spew of data

        if(SerialDebug) {
            Serial.println("Calibration values: ");
            Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);

            attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for intPin output of MPU9250
        }     
    }else{
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        return false;
    }
    return true;
}

void myinthandler()
{
  intFlag = true;
}

