
#include "CamSerial.h"
HardwareSerial CamSerial(1);
void CamSerialClass::InitCamSerial() {
  CamSerial.begin(115200, SERIAL_8N1, S3_RX_PIN, S3_TX_PIN);
  Serial.println("=====================================");
  Serial.println("✅ ESP32-S3 主控已成功启动！");
  Serial.println("🎧 正在监听来自 ESP32-CAM 的动作指令...");
  Serial.println("=====================================");
}

String CamSerialClass::readCamSerial(){
  // 非阻塞读取：逐字符读入缓冲区，遇到 '\n' 才返回完整命令
  while (CamSerial.available()) {
    char c = CamSerial.read();
    if (c == '\n') {
      String cmd = _rxBuffer;
      _rxBuffer = "";
      cmd.trim();
      if (cmd.length() > 0) {
        Serial.print("💡 S3 成功接收: [");
        Serial.print(cmd);
        Serial.println("]");
        return cmd;
      }
      return "";
    }
    _rxBuffer += c;
    // 防止缓冲区溢出（异常情况）
    if (_rxBuffer.length() > 64) {
      _rxBuffer = "";
    }
  }
  return "";
}

String CamSerialClass::SwitchCamMode(String cmd) {
    // 强制转为大写，增加鲁棒性
    cmd.toUpperCase(); 

    if (cmd.indexOf("WALK") >= 0) {
        return "WALK";
    
    } 
    else if (cmd.indexOf("STAND") >= 0) {
        return "STAND";
      
    } 
    else if (cmd.indexOf("LIE") >= 0) {  
        return "REST";
     
    }else
    {
        return "";
    }
}

void CamSerialClass::sendImuDataToCam(float current_roll, float current_pitch, float current_yaw) {
  // 每隔 100ms 向 CAM 发送一次模拟数据
  static unsigned long lastImuTime = 0;
  Serial.printf("准备发送 IMU 数据 -> Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", current_roll, current_pitch, current_yaw);
  if (millis() - lastImuTime > 100) {
    
    // 【核心修改】：生成随机浮点数模拟姿态角
    // 模拟横滚 (Roll) 和 俯仰 (Pitch) 在 -90.00° 到 90.00° 之间随机跳动
    // 按照约定格式拼接字符串: "IMU:横滚,俯仰,偏航"
    // String(变量, 1) 代表保留 1 位小数，这跟前端网页的处理逻辑完美契合
    String imuPayload = "IMU:" + String(current_roll, 1) + "," + 
                        String(current_pitch, 1) + "," + 
                        String(current_yaw, 1);
                        
    // 通过与 CAM 相连的串口发送出去 (自带 \n 换行符)
    CamSerial.println(imuPayload);
    
    lastImuTime = millis();
  }
}

void CamSerialClass::sendBatteryDataToCam(float voltage, float current, int percentage) {
  static unsigned long lastBatTime = 0;
  if (millis() - lastBatTime > 1000) {
    String batPayload = "BAT:" + String(voltage, 1) + "," +
                        String(current, 1) + "," +
                        String(percentage);
    CamSerial.println(batPayload);
    lastBatTime = millis();
  }
}