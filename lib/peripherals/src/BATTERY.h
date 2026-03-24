#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// 引脚定义
#define PIN_VOLTAGE 36  // SVP
#define PIN_CURRENT 39  // SVN
#define PIN_RELAY   27  // 继电器控制

// 传感器硬件常数
const float V_REF = 3.3f;
const float ADC_RES = 4095.0f;
const float VOLT_DIVIDER = 5.463f;   // 你校准后的分压系数
const float CURR_SENSITIVITY = 0.066f; // 66mV/A

// 逻辑阈值
const float MAX_CURRENT_PROTECT = 15.0f; // 15A 保护
const float VOLT_CUTOFF = 5.4f;          // 电池物理截止电压

// 外部可调用函数
void BATTERY_Init();                // 初始化引脚和采样环境
void BATTERY_Update();              // 核心逻辑：采样、滤波、保护判断

// 获取实时数据的接口
float BATTERY_GetVoltage();         // 获取滤波后的电压
float BATTERY_GetCurrent();         // 获取滤波后的电流
int   BATTERY_GetPercentage();      // 获取电量百分比
bool  BATTERY_IsSafetyCutoff();     // 是否触发了安全断开

#endif
