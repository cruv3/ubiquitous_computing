#include <Arduino.h>
#define PRESSURE_SENSOR_PIN 32  // Any ADC pin (32, 33, 34, 35, 36, 39)

void print_sensor_data(int value) {
    // Create a properly formatted JSON string
    String jsonData = "{";
    jsonData += "\"analogRead\": " + String(value);
    jsonData += "}";

    Serial.println(jsonData);  // Print JSON data
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);  // 12-bit ADC (0-4095)
}

void loop() {
    print_sensor_data(analogRead(PRESSURE_SENSOR_PIN));
    delay(500);
}
