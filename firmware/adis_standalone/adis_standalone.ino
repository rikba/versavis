#include "versavis_configuration.h"
#include "drivers/Adis16448.h"
#include "Arduino.h"
#include <ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>

const int kImuDataReadyInPin = 38;
const int kChipSelectPin = 10;
Adis16448 imu(kChipSelectPin);
volatile bool imu_ready = false;
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

void setup() {
  nh.initNode();
  while (!SerialUSB)
    ;
  nh.loginfo("Node initialized.");
  nh.loginfo("Config IMU...");
  SPI.begin();
  imu.setDefault();
  //imu.regWrite(GLOB_CMD, 0x8);
  //delay(300);
  imu.regWrite(MSC_CTRL, 0x56);
  delay(20);
  imu.regWrite(GPIO_CTRL, 0x202);
  delay(20);
  imu.regWrite(SMPL_PRD, 0x001);
  delay(20);
  imu.regWrite(SENS_AVG, 0x107);
  delay(20);

  int16_t flash_cnt = imu.regRead(FLASH_CNT);
  char flash_cnt_str[25];
  sprintf(flash_cnt_str, "Flash count: %d", flash_cnt);
  nh.loginfo(flash_cnt_str);

  int16_t lot_id1 = imu.regRead(LOT_ID1);
  char lot_id1_str[25];
  sprintf(lot_id1_str, "LOT_ID1: %d", lot_id1);
  nh.loginfo(lot_id1_str);
  if ((lot_id1 & 0xFF) < 0x22) {
    nh.logwarn("Silicon anomaly detected. https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16448-Silicon-Anomaly.pdf");
  }

  int16_t lot_id2 = imu.regRead(LOT_ID2);
  char lot_id2_str[25];
  sprintf(lot_id2_str, "LOT_ID2: %d", lot_id2);
  nh.loginfo(lot_id2_str);

  int16_t prod_id = imu.regRead(PROD_ID);
  char prod_id_str[25];
  sprintf(prod_id_str, "PROD_ID: %d", prod_id);
  nh.loginfo(prod_id_str);

  int16_t serial_num = imu.regRead(SERIAL_NUM);
  char serial_num_str[25];
  sprintf(serial_num_str, "SERIAL_NUM: %d", serial_num);
  nh.loginfo(serial_num_str);

  nh.loginfo("Configure interrupts...");
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(kImuDataReadyInPin), imuReady, RISING);
  interrupts();

  nh.advertise(imu_pub);
  char pub_id_str[25];
  sprintf(pub_id_str, "PUB ID: %d", imu_pub.id_);
  nh.loginfo(pub_id_str);
}

void imuReady() { imu_ready = true; }

void loop() {
  // Process IMU data.
  if (imu_ready) {
     publishIMUData();
     imu_ready = false;
  }

  // Toggle IMU LED.
  toggleImuLed();
  nh.spinOnce();
}

void toggleNthBit(int pos, int16_t* word) { *word = *word ^ (1 << pos); }

void toggleImuLed() {
  static unsigned long previous_led_time = 0;
  const unsigned long interval = 1000;  // ms to toggle LED.
  unsigned long current_led_time = millis();
  if (current_led_time - previous_led_time >= interval) {
    previous_led_time = current_led_time;
    int16_t reg = imu.regRead(GPIO_CTRL);  // GPIO reg.
    toggleNthBit(9, &reg);  // GPIO 2 data level.
    imu.regWrite(GPIO_CTRL, reg);
  }
}

void publishIMUData() {
  // static char spi_timing[25];
  // static char crc_timing[25];
  // static char scale_timing[25];
  // static char pub_timing[25];
  // static uint32_t spi_start, spi_stop, crc_start, crc_stop, scale_start, scale_stop, pub_start, pub_stop;

  // spi_start = micros();
  int16_t* imu_data = imu.sensorReadAllCRC();
  // spi_stop = micros();

  // crc_start = micros();
  auto success = (imu_data[12] == imu.checksum(imu_data));
  // crc_stop = micros();

  if (success) {
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu";
    imu_msg.header.seq++;

    // scale_start = micros();
    imu_msg.angular_velocity.x = imu.gyroScale(imu_data[1]);
    imu_msg.angular_velocity.y = imu.gyroScale(imu_data[2]);
    imu_msg.angular_velocity.z = imu.gyroScale(imu_data[3]);

    imu_msg.linear_acceleration.x = imu.accelScale(imu_data[4]);
    imu_msg.linear_acceleration.y = imu.accelScale(imu_data[5]);
    imu_msg.linear_acceleration.z = imu.accelScale(imu_data[6]);

    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    // scale_stop = micros();

    // pub_start = micros();
    imu_pub.publish(&imu_msg);
    // pub_stop = micros();
  } else {
    nh.logerror("Checksum error.");
  }
  if (imu_data[0] && !(imu_data[0] & (1 << 7))) {
    char diag_stat_str[25];
    sprintf(diag_stat_str, "DIAG_STAT: %d", imu_data[0]);
    nh.logerror(diag_stat_str);
    if (imu_data[0] & (1 << 0)) {
      nh.logerror("Magnetometer functional test failure.");
    }
    if (imu_data[0] & (1 << 1)) {
      nh.logerror("Barometer functional test failure.");
    }
    if (imu_data[0] & (1 << 2)) {
      nh.logerror("Flash update failure.");
    }
    if (imu_data[0] & (1 << 3)) {
      nh.logerror("SPI communication failure.");
    }
    if (imu_data[0] & (1 << 4)) {
      nh.logerror("Sensor overrange.");
    }
    if (imu_data[0] & (1 << 5)) {
      nh.logerror("Self-test diagnostic error flag.");
    }
    if (imu_data[0] & (1 << 6)) {
      nh.logerror("Flash test, checksum flag.");
    }
    if (imu_data[0] & (1 << 7)) {
      nh.logerror("New data, xMAGN_OUT/BARO_OUT.");
    }
    if (imu_data[0] & (1 << 8)) {
      nh.logerror("Alarm 1.");
    }
    if (imu_data[0] & (1 << 9)) {
      nh.logerror("Alarm 2.");
    }
    if (imu_data[0] & (1 << 10)) {
      nh.logerror("X-axis gyroscope self-test failure.");
    }
    if (imu_data[0] & (1 << 11)) {
      nh.logerror("Y-axis gyroscope self-test failure.");
    }
    if (imu_data[0] & (1 << 12)) {
      nh.logerror("Z-axis gyroscope self-test failure.");
    }
    if (imu_data[0] & (1 << 13)) {
      nh.logerror("X-axis accelerometer self-test failure");
    }
    if (imu_data[0] & (1 << 14)) {
      nh.logerror("Y-axis accelerometer self-test failure");
    }
    if (imu_data[0] & (1 << 15)) {
      nh.logerror("Z-axis accelerometer self-test failure");
    }
  }

  // sprintf(spi_timing, "SPI: %d us", spi_stop - spi_start);
  // sprintf(crc_timing, "CRC: %d us", crc_stop - crc_start);
  // sprintf(scale_timing, "Scale: %d us", scale_stop - scale_start);
  // sprintf(pub_timing, "Publish: %d us", pub_stop - pub_start);
  // nh.loginfo(spi_timing);
  // nh.loginfo(crc_timing);
  // nh.loginfo(scale_timing);
  // nh.loginfo(pub_timing);
}
