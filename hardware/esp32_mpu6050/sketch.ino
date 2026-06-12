#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("{\"error\":\"mpu6050_not_found\"}");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(200);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long ts = millis();

  float ax_g = accel.acceleration.x / 9.80665f;
  float ay_g = accel.acceleration.y / 9.80665f;
  float az_g = accel.acceleration.z / 9.80665f;

  float gx_dps = gyro.gyro.x * 57.29578f;
  float gy_dps = gyro.gyro.y * 57.29578f;
  float gz_dps = gyro.gyro.z * 57.29578f;

  Serial.print("{\"ts_ms\":");
  Serial.print(ts);
  Serial.print(",\"ax_g\":");
  Serial.print(ax_g, 4);
  Serial.print(",\"ay_g\":");
  Serial.print(ay_g, 4);
  Serial.print(",\"az_g\":");
  Serial.print(az_g, 4);
  Serial.print(",\"gx_dps\":");
  Serial.print(gx_dps, 3);
  Serial.print(",\"gy_dps\":");
  Serial.print(gy_dps, 3);
  Serial.print(",\"gz_dps\":");
  Serial.print(gz_dps, 3);
  Serial.print(",\"temp_c\":");
  Serial.print(temp.temperature, 2);
  Serial.println("}");

  delay(100);
}