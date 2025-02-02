#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

void print_roll_pitch_yaw();
void print_calibration();
void print_sensor_data();
void print_matlab_data();

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();
    delay(5000);
    mpu.calibrateMag();

    //print_calibration();
    //print_calibration();
    mpu.verbose(false);
}


void loop() {
    bool value = false;
    if(!value) {
        value = true;
    }else{
        return;
    }

    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            //print_sensor_data();
            print_matlab_data();
            prev_ms = millis();
        }
    }
}

void print_matlab_data() {
    Serial.print(mpu.getGyroX(), 2); Serial.print(",");
    Serial.print(mpu.getGyroY(), 2); Serial.print(",");
    Serial.print(mpu.getGyroZ(), 2); Serial.print(",");
    Serial.print(mpu.getAccX(), 2); Serial.print(",");
    Serial.print(mpu.getAccY(), 2); Serial.print(",");
    Serial.println(mpu.getAccZ(), 2);  // Use println only at the end
}


void print_sensor_data() {
    String jsonData = "{";
    jsonData += "\"gyro\": [" + String(mpu.getGyroX(), 2) + ", " + String(mpu.getGyroY(), 2) + ", " + String(mpu.getGyroZ(), 2) + "], ";
    jsonData += "\"accel\": [" + String(mpu.getAccX(), 2) + ", " + String(mpu.getAccY(), 2) + ", " + String(mpu.getAccZ(), 2) + "], ";
    jsonData += "\"mag\": [" + String(mpu.getMagX(), 2) + ", " + String(mpu.getMagY(), 2) + ", " + String(mpu.getMagZ(), 2) + "], ";
    jsonData += "\"yaw\": " + String(mpu.getYaw(), 2) + ", ";
    jsonData += "\"pitch\": " + String(mpu.getPitch(), 2) + ", ";
    jsonData += "\"roll\": " + String(mpu.getRoll(), 2);
    jsonData += "}";

    Serial.println(jsonData);  // Print JSON data
}

void print_roll_pitch_yaw() {
    String jsonData = "{";
    jsonData += "\"yaw\": " + String(mpu.getYaw(), 2) + ", ";
    jsonData += "\"pitch\": " + String(mpu.getPitch(), 2) + ", ";
    jsonData += "\"roll\": " + String(mpu.getRoll(), 2);
    jsonData += "}";

    Serial.println(jsonData);  // Print JSON data
}


void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}