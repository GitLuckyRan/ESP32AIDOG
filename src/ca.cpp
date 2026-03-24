#include <Arduino.h>

// 引脚重定义
#define PIN_VOLTAGE 36  // SVP
#define PIN_CURRENT 39  // SVN
#define PIN_RELAY   27   // 继电器

// 传感器参数
const float V_REF = 3.3;             // ADC 参考电压 (取决于电路供电)
const float ADC_RESOLUTION = 4095.0; // 12位分辨率
const float VOLT_DIVIDER = 5.0;      // 电压分压系数
const float CURR_OFFSET = 2.5;       // 0A 时的输出电压 (2.5V)
const float CURR_SENSITIVITY = 0.066; // 灵敏度 66mV/A

void setup() {
    Serial.begin(115200);
    pinMode(PIN_RELAY, OUTPUT);
    
    // 设置 ADC 衰减 (11dB 允许测量 0-3.1V 左右)
    analogSetAttenuation(ADC_11db); 
}

void loop() {
    // 1. 读取原始值
    int rawV = analogRead(PIN_VOLTAGE);
    int rawI = analogRead(PIN_CURRENT);

    // 2. 转换为实际引脚电压
    float vAtPin = (rawV / ADC_RESOLUTION) * V_REF;
    float iAtPin = (rawI / ADC_RESOLUTION) * V_REF;

    // 3. 换算为物理量
    float actualVoltage = vAtPin * VOLT_DIVIDER;
    // 4. 根据你的注释：电流增加时电压下降
    float actualCurrent = (CURR_OFFSET - iAtPin) / CURR_SENSITIVITY;

    // 打印数据
    Serial.printf("Battery: %.2fV | Load: %.2fA\n", actualVoltage, actualCurrent);

    // 简易逻辑控制：例如电流超过 10A 自动关断继电器（保护）
    if (actualCurrent > 20) {
        Serial.printf("DISCONNECTED - Battery: %.2fV | Load: %.2fA\n", actualVoltage, actualCurrent);
        digitalWrite(PIN_RELAY, HIGH); 
    } else {
        digitalWrite(PIN_RELAY, LOW);
    }

    delay(200);
}