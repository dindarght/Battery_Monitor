#include <Arduino.h>

// Constants
const int ADC_PIN = 34; // GPIO pin connected to the battery voltage divider
const float ADC_MAX_VALUE = 4095.0; // Maximum value for a 12-bit ADC
const float ADC_VOLTAGE_REF = 3.3; // Reference voltage for ADC
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Ratio of the voltage divider (assumed 1:1 for simplicity)
const float BATTERY_MAX_VOLTAGE = 4.2; // Maximum voltage of the LiPo battery
const float BATTERY_MIN_VOLTAGE = 3.0; // Minimum voltage of the LiPo battery

class BatteryManager {
  public:
    BatteryManager(int pin) : _pin(pin) {}

    // Function to read the raw ADC value
    int readADCRaw() {
      return analogRead(_pin);
    }

    // Function to convert ADC value to battery voltage
    float getBatteryVoltage() {
      int adcValue = readADCRaw();
      float voltage = (adcValue / ADC_MAX_VALUE) * ADC_VOLTAGE_REF * VOLTAGE_DIVIDER_RATIO;
      return voltage;
    }

    // Function to convert battery voltage to battery percentage
    int getBatteryPercentage() {
      float voltage = getBatteryVoltage();
      float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100;
      percentage = constrain(percentage, 0, 100); // Ensure percentage is within 0-100%
      return (int)percentage;
    }

    // Function to check if battery is low
    bool isBatteryLow() {
      return getBatteryVoltage() < BATTERY_MIN_VOLTAGE;
    }

    // Function to send battery status via Serial
    void sendBatteryStatus() {
      float voltage = getBatteryVoltage();
      int percentage = getBatteryPercentage();
      Serial.print("Battery Voltage: ");
      Serial.print(voltage);
      Serial.print(" V, ");
      Serial.print("Battery Percentage: ");
      Serial.print(percentage);
      Serial.println(" %");
    }

  private:
    int _pin;
};

BatteryManager batteryManager(ADC_PIN);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
}

void loop() {
  batteryManager.sendBatteryStatus();
  delay(5000); // Wait for 5 seconds before the next measurement
}