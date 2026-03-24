#ifndef CAMSERIAL_H
#define CAMSERIAL_H

#include <HardwareSerial.h>
#include <Arduino.h>

#define S3_RX_PIN 5 // 接 CAM 的 13
#define S3_TX_PIN 16 // 接 CAM 的 14


class CamSerialClass {
public:
    void InitCamSerial() ;
    String readCamSerial();
    void sendImuDataToCam(float current_roll, float current_pitch, float current_yaw);
    String SwitchCamMode(String cmd);
    String cmd = "";

};
extern CamSerialClass CAM;
#endif 