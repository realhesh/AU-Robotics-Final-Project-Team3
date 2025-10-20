#include <MPU9250.h>

MPUYaw imu(0x68, 0x0C, 21, 22);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing MPU9250...");
  imu.begin();
}

void loop() {
  if (imu.update()) {
    Serial.print("Yaw: ");
    Serial.println(imu.getYaw());
  }
}
