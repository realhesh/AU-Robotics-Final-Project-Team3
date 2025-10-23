#include "MPU6050Yaw.h"

MPU6050Yaw imu(true); // true = Madgwick, false = Mahony

void setup() {
  Serial.begin(115200);
  imu.begin();

  Serial.println("=== MPU6050 YAW TRACKER ===");
  Serial.println(imu.useMadgwick ? "Algorithm: Madgwick" : "Algorithm: Mahony");
  if (imu.useMadgwick) {
    Serial.print("Beta: "); Serial.println(imu.beta, 3);
  } else {
    Serial.print("Kp: "); Serial.print(imu.Kp);
    Serial.print(" | Ki: "); Serial.println(imu.Ki);
  }
  Serial.println("============================\n");
}

void loop() {
  imu.update();
  float yaw = imu.getYaw();

  Serial.print("Yaw: ");
  Serial.print(yaw, 2);
  Serial.println("°");

  delay(10);
}
