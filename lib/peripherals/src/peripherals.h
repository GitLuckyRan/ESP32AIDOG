#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>

/*
 * 超声波传感器设置
 */
#define MAX_DISTANCE 200

/**
 * Peripherals 类：纯粹的硬件管理类
 * 移除了 StatefulService 继承、持久化和 Web 接口
 */
class Peripherals {
public:
    Peripherals();

    // 基础生命周期函数
    void begin();
    void update();

    // I2C 总线管理
    void updatePins(int8_t sda, int8_t scl, uint32_t frequency = 100000);
    void scanI2C();

    // 传感器数据读取逻辑
    bool readImu();
    bool readMag();
    bool readBMP();
    void readSonar();

    // 数据获取接口
    float angleX();
    float angleY();
    float angleZ();
    float leftDistance();
    float rightDistance();

    float getHeading() { return _mag_heading; }

    // 硬件校准
    bool calibrateIMU();
    void fuseHeading();


    // void calibrateMag();

    // 公共状态变量
    float _left_distance {MAX_DISTANCE};
    float _right_distance {MAX_DISTANCE};
    bool i2c_active = false;
    float _mag_heading;
private:
    float _fusedYaw = 0;       // 融合后的最终航向
    float _yawOffset = 0;      // IMU与磁北的初始差值
    bool _firstFusion = true;  // 是否是第一次融合


};

#endif

