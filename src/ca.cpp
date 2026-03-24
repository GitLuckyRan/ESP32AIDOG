// #include <Arduino.h>
// #include "BATTERY.h"
// // 引脚重定义
// // 自动校准函数


// void setup() {
//     Serial.begin(115200);
//     BATTERY_Init();
//     delay(1000);

// }

// void loop() {
//     // 1. 读取原始值
//     BATTERY_Update();
//     // 2. 输出当前电压、电流和电量百分比
//     // BATTERY_GetVoltage();
//     // BATTERY_GetCurrent();
//     // BATTERY_GetPercentage(); 
//     // BATTERY_IsSafetyCutoff();
//     Serial.printf("电压: %.2f V, 电流: %.2f A, 电量: %d%%, 安全断开: %s\n", 
//                   BATTERY_GetVoltage(), BATTERY_GetCurrent(), BATTERY_GetPercentage(),
//                   BATTERY_IsSafetyCutoff() ? "是" : "否");


//     delay(200);
// }
