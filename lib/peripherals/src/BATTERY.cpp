#include "BATTERY.h"

// 内部变量
static float _currentOffset = 2.332f; 
static float _filteredVoltage = 8.4f;
static float _filteredCurrent = 0.0f;
static bool  _isCutoff = false;

// 查表法：LiPo 2S 电压对应电量百分比
float calculatePercentage(float v) {
    const float vTable[] = {6.60, 6.80, 6.95, 7.10, 7.24, 7.40, 7.55, 7.72, 7.90, 8.10, 8.40};
    const int pTable[]   = {0,   10,   20,   30,   40,   50,   60,   70,   80,   90,   100};
    if (v >= 8.40f) return 100;
    if (v <= 6.60f) return 0;
    for (int i = 0; i < 10; i++) {
        if (v < vTable[i+1]) {
            return pTable[i] + (v - vTable[i]) * (pTable[i+1] - pTable[i]) / (vTable[i+1] - vTable[i]);
        }
    }
    return 0;
}

void BATTERY_Init() {
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW); // 初始吸合/接通
    analogSetAttenuation(ADC_11db);
    _isCutoff = false;
}


void BATTERY_Update() {
    // 1. 原始采样
    float vRaw = (analogRead(PIN_VOLTAGE) / ADC_RES) * V_REF * VOLT_DIVIDER;
    float iPin = (analogRead(PIN_CURRENT) / ADC_RES) * V_REF;
    float iRaw = (_currentOffset - iPin) / CURR_SENSITIVITY;
    if (iRaw < 0) iRaw = 0;

    // 2. 软件滤波 (EMA)，防止机器人运动瞬时压降导致的误触发
    _filteredCurrent = (0.2f * iRaw) + (0.8f * _filteredCurrent);

    if (_filteredCurrent < 0.6f) {
        // 使用较慢的滤波，消除 ADC 抖动
        _filteredVoltage = vRaw;
    }

    // 3. 安全保护逻辑
    // 电流超过 15A 或 电池电压低于截止线
    if (_filteredCurrent > MAX_CURRENT_PROTECT || _filteredVoltage < VOLT_CUTOFF) {
        digitalWrite(PIN_RELAY, HIGH); // 断开
        _isCutoff = true;
    }
}

float BATTERY_GetVoltage() { return _filteredVoltage; }
float BATTERY_GetCurrent() { return _filteredCurrent; }
int   BATTERY_GetPercentage() { return (int)calculatePercentage(_filteredVoltage); }
bool  BATTERY_IsSafetyCutoff() { return _isCutoff; }