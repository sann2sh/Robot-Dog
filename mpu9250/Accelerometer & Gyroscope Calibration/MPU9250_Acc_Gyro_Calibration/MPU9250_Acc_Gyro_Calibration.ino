#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);

#define GRAVITY 9.802

float accel[3];
float gyro[3];
int sample_times = 1;

float acc_cal[3] = {0.0, 0.0, 0.0};
float gyro_cal[3] = {0.0, 0.0, 0.0};

void read_sensors() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();
  
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}


// calculate accelerometer and gyroscope bias
void calibrate_imu_bias() {
  double accel_sum[3] = {0, 0, 0};
  double gyro_sum[3] = {0, 0, 0};
  for (int i = 0; i < 1000; i++)
  {
    read_sensors();
    for (int j = 0; j < 3; j++)
    {
      accel_sum[j] += accel[j];
      gyro_sum[j] += gyro[j];
    }
    delay(4);
  }
  for (int j = 0; j < 3; j++)
  {
    accel_sum[j] /= 1000;
    gyro_sum[j] /= 1000;
  }
  accel_sum[2] = accel_sum[2] + GRAVITY;
  IMU.setAccelCalX(accel_sum[0], 1.0);
  IMU.setAccelCalY(accel_sum[1], 1.0);
  IMU.setAccelCalZ(accel_sum[2], 1.0);
  IMU.setGyroBiasX_rads(gyro_sum[0]);
  IMU.setGyroBiasY_rads(gyro_sum[1]);
  IMU.setGyroBiasZ_rads(gyro_sum[2]);
}


void setup() {
  Serial.begin(2000000);
  while (!Serial) yield();

  // start communication with IMU 
  IMU.begin();
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
   IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
  //IMU.calibrateGyro();
  //IMU.calibrateAccel();

  calibrate_imu_bias();

  Serial.println(IMU.getAccelBiasX_mss());
  Serial.println(IMU.getAccelScaleFactorX());
}

void loop() {
  read_sensors();
  for (int i = 0; i < 3; i++) {
    acc_cal[i] += accel[i];
    gyro_cal[i] += gyro[i];
  }
  Serial.print(acc_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[2] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.println(gyro_cal[2] / sample_times, 7);
  sample_times++;
  
}
