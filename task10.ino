#include <Wire.h>

#define MPU6050_ADDR 0x68  // MPU6050 I2C address
#define PWR_MGMT_1 0x6B    // Power management register
#define GYRO_XOUT_H 0x43   // Gyroscope X-axis data high byte
#define GYRO_YOUT_H 0x45   // Gyroscope Y-axis data high byte
#define GYRO_ZOUT_H 0x47   // Gyroscope Z-axis data high byte

float gyroZOffset = 0;
float yaw = 0;
unsigned long prevTime = 0;
float dt = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Wake up the MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  // Calibrate the gyroscope (Z-axis)
  calibrateGyroZ();

  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;  // Time difference in seconds

  float gyroZ = readGyroZ() - gyroZOffset;  // Read and offset the Z-axis gyroscope data

  yaw += gyroZ * dt;  // Integrate gyroscope data to obtain yaw angle

  prevTime = currentTime;

  // Print the yaw angle
  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(100);  // Adjust delay as needed
}

void calibrateGyroZ() {
  int numReadings = 1000;
  for (int i = 0; i < numReadings; i++) {
    gyroZOffset += readGyroZ();
    delay(3);
  }
  gyroZOffset /= numReadings;
}

float readGyroZ() {
  int16_t gyroZRaw = read16BitData(GYRO_ZOUT_H);
  float gyroZ = gyroZRaw / 131.0;  // Convert raw data to degrees per second (dps)
  return gyroZ;
}

int16_t read16BitData(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);

  int16_t data = Wire.read() << 8 | Wire.read();
  return data;
}

