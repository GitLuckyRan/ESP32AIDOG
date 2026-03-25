#include <Arduino.h>
#include <peripherals.h>
#include <Wire.h>
#include <motion.h>
#include <servo_controller.h>
#include <CamSerial.h>
#include "BATTERY.h"
// 创建外设管理对象
Peripherals  peripherals;

ServoController servoController;
MotionService motionService;
CamSerialClass CAM;




void SpotControlLoopEntry(void *) {
    Serial.println("[main] Control task starting");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    CAM.InitCamSerial();
    peripherals.begin();
    peripherals.update();

    Serial.printf("左超声波距离: %.2f cm, 右超声波距离: %.2f cm\n", 
                  peripherals.leftDistance(), peripherals.rightDistance());
    
    Serial.printf("IMU 角度 -> X: %.2f°, Y: %.2f°, Z: %.2f°\n", 
                  peripherals.angleX(), peripherals.angleY(), peripherals.angleZ());
                  
    Serial.printf("磁力计航向: %.2f°\n", peripherals.getHeading());
    servoController.begin();
    motionService.begin();
    // peripherals.calibrateIMU();

    motionService.update(&peripherals);  // 强制运行一次，生成 target_angles
    servoController.setAngles(motionService.getAngles());
    servoController.update();
    servoController.activate();
    

    for (;;) {
        WARN_IF_SLOW(SpotControlLoopEntry, 20);
        String cmd = CAM.SwitchCamMode(CAM.readCamSerial());
        if (cmd.length() > 0) {
            motionService.handleMode(cmd);
        }
        peripherals.update();
        motionService.update(&peripherals);
        servoController.setAngles(motionService.getAngles());
        servoController.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    // 1. 初始化串口通信
    Serial.begin(115200);
    BATTERY_Init();
    BATTERY_Update();
    delay(2000); // 等待串口稳定
    Serial.println("--- SpotMicro 传感器自检程序 ---");
    // CAM.InitCamSerial();
    // peripherals.begin();
    // peripherals.update();

    // Serial.printf("左超声波距离: %.2f cm, 右超声波距离: %.2f cm\n", 
    //               peripherals.leftDistance(), peripherals.rightDistance());
    
    // Serial.printf("IMU 角度 -> X: %.2f°, Y: %.2f°, Z: %.2f°\n", 
    //               peripherals.angleX(), peripherals.angleY(), peripherals.angleZ());
                  
    // Serial.printf("磁力计航向: %.2f°\n", peripherals.getHeading());
    
    
    xTaskCreatePinnedToCore(SpotControlLoopEntry, "Control task", 8192*2, nullptr, 5, nullptr, 1);
}




void loop() {
//   CAM.SwitchCamMode(CAM.readCamSerial());
//   peripherals.update();
  CAM.sendImuDataToCam(peripherals.angleY(), peripherals.angleX(), peripherals.angleZ());
  CAM.sendBatteryDataToCam(BATTERY_GetVoltage(), BATTERY_GetCurrent(), BATTERY_GetPercentage());
  BATTERY_Update();
    // 2. 输出当前电压、电流和电量百分比
    // BATTERY_GetVoltage();
    // BATTERY_GetCurrent();
    // BATTERY_GetPercentage(); 
    // BATTERY_IsSafetyCutoff();
  Serial.printf("电压: %.2f V, 电流: %.2f A, 电量: %d%%, 安全断开: %s\n", 
                  BATTERY_GetVoltage(), BATTERY_GetCurrent(), BATTERY_GetPercentage(),
                  BATTERY_IsSafetyCutoff() ? "是" : "否");
  delay(1000); // 每 1000ms 发送一次数据
  // 1. 获取外设数据（保持习惯）

}


