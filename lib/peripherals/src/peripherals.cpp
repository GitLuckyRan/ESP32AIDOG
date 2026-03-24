#include <peripherals.h>
#include <Wire.h>
#include <NewPing.h> // Arduino 标准超声波库
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

#include <QMC5883LCompass.h>




// 定义超声波引脚（根据实际接线修改）
#define USS_LEFT_TRIG  33
#define USS_LEFT_ECHO  34
#define USS_RIGHT_TRIG 32
#define USS_RIGHT_ECHO 35

#define I2C_SDA 18
#define I2C_SCL 19


#define PWM_FREQ 50


uint8_t devStatus;      // 器件状态（0 = 成功，!0 = 错误）
uint16_t packetSize;    // 预期的 DMP 数据包大小
uint8_t fifoBuffer[64]; // FIFO 缓冲区
Quaternion q;           // 四元数 [w, x, y, z]
VectorFloat gravity;    // 重力矢量
float ypr[3];           // [yaw, pitch, roll] 偏航/俯仰/翻滚


MPU6050 mpu;
QMC5883LCompass compass;


// 初始化超声波对象
NewPing sonarLeft(USS_LEFT_TRIG, USS_LEFT_ECHO, MAX_DISTANCE);
NewPing sonarRight(USS_RIGHT_TRIG, USS_RIGHT_ECHO, MAX_DISTANCE);


// 新增：存储校准后的偏移值
int16_t xAccelOffset, yAccelOffset, zAccelOffset;
int16_t xGyroOffset, yGyroOffset, zGyroOffset;



Peripherals::Peripherals() {
    // 构造函数：初始化基础变量
    _left_distance = MAX_DISTANCE;
    _right_distance = MAX_DISTANCE;
    i2c_active = false;
    ypr[0] = ypr[1] = ypr[2] = 0;
}

void Peripherals::begin() 
{
    // 1. 初始化 I2C 总线（使用默认引脚或之前设定的引脚）
    // ESP32 默认 I2C: SDA=21, SCL=22 (根据实际修改)
    i2c_active = Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    Serial.println(i2c_active ? "I2C 总线初始化成功" : "I2C 总线初始化失败");

    if (i2c_active) {
        Serial.println("I2C 总线已启动");
        /**
         * 关键步骤：开启 I2C 旁路模式
         * 作用：让 ESP32 可以直接穿过 MPU6050 与 HMC5883L 磁力计通信
         */
        mpu.initialize();


        devStatus = mpu.dmpInitialize();
        mpu.setI2CBypassEnabled(true);
        Serial.println(F("I2C 旁路模式已开启（磁力计现在可见）"));
        delay(50);
        compass.init();
        compass.setCalibration(-847, 304, 192, 1310, -207, 2098);
        compass.setMagneticDeclination(8,16); // 设置磁偏角（根据你所在位置调整）
        Serial.println("磁力计初始化完成");
        

        if (devStatus == 0) {
            // 开启 DMP
            // mpu.setDMPEnabled(true);
            // mpu.CalibrateAccel(6);
            // mpu.CalibrateGyro(6);

            mpu.setXAccelOffset(-5050);
            mpu.setYAccelOffset(-1968);
            mpu.setZAccelOffset(7212);

            // 2. 注入你刚才获取的 Gyro 偏移量
            mpu.setXGyroOffset(47);
            mpu.setYGyroOffset(26);
            mpu.setZGyroOffset(60);
            mpu.setDMPEnabled(true);
            
            Serial.println(F("IMU 偏移量已注入，DMP 快速启动成功！"));

            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // 3. 打印查看
            Serial.println("--- 静态补偿后原始数据检查 ---");
            Serial.printf("Accel Raw: %d, %d, %d\n", ax, ay, az);
            Serial.printf("Gyro Raw: %d, %d, %d\n", gx, gy, gz);

            // 执行高精度校准

            packetSize = mpu.dmpGetFIFOPacketSize();
            Serial.println(F("DMP 初始化成功"));
        } else {
            Serial.print(F("DMP 初始化失败 (代码 "));
            Serial.print(devStatus);
            Serial.println(F(")"));
            }

        // delay(500);
        // pca9685.begin();
        // pca9685.setPWMFreq(PWM_FREQ);
        // pca9685.setOscillatorFrequency(27000000);
        // Serial.println("✅ PCA9685-初始化完成（脉冲校准模式）");


        }
       

     else {
        Serial.println("I2C 总线启动失败！");
    }

    // 3. 超声波传感器准备就绪提示
    Serial.println("双超声波传感器已就绪");
}



void Peripherals::update() {
    // 使用定时宏或 millis() 控制不同传感器的采集频率
    
    // 每 20ms 读取一次 IMU (用于快速平衡控制)
    static uint32_t lastImuMicros = 0;
    if (millis() - lastImuMicros >= 20) {
        lastImuMicros = millis();
        readImu();
    }

    // 每 100ms 读取一次磁力计 (用于航向校准)
    static uint32_t lastMagMicros = 0;
    if (millis() - lastMagMicros >= 100) {
        lastMagMicros = millis();
        readMag();
    }

    // 每 500ms 读取一次气压计和超声波 (用于高度和避障)
    static uint32_t lastSlowMicros = 0;
    if (millis() - lastSlowMicros >= 500) {
        lastSlowMicros = millis();
        readBMP();
        readSonar();
    }
}

/* 内部读取函数的具体实现 */

bool Peripherals::readImu() {
    if (!i2c_active) return false;

    // 获取最新的 FIFO 数据包
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // 解析四元数
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // 获取重力矢量
        mpu.dmpGetGravity(&gravity, &q);
        // 获取欧拉角（弧度转角度）
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        fuseHeading() ; // 融合磁力计和 IMU 的航向数据

        // 更新类内部变量
        // 注意：ypr[0] 是 Yaw，ypr[1] 是 Pitch，ypr[2] 是 Roll
        return true;
    }
    return false;
}


// 角度接口实现
float Peripherals::angleX() { return ypr[2] * 180 / M_PI; } // Roll
float Peripherals::angleY() { return ypr[1] * 180 / M_PI; } // Pitch
float Peripherals::angleZ() { return ypr[0] * 180 / M_PI; } // Yaw



void Peripherals::readSonar() {
    // 读取左侧距离
    _left_distance = sonarLeft.ping_cm();
    
    // 稍微延迟避免声波交叉干扰
    delay(50); 
    
    // 读取右侧距离
    _right_distance = sonarRight.ping_cm();
}

// 数据接口实现
float Peripherals::leftDistance() { return _left_distance; }
float Peripherals::rightDistance() { return _right_distance; }



bool Peripherals::calibrateIMU() {
  Serial.println("========== 手动校准开始 ==========");
  Serial.println("请保持 MPU 完全静止！3s 后开始...");
  delay(3000);

  // 先清零偏移
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(100);

  int16_t ax, ay, az, gx, gy, gz;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

  const int N = 1500;  // 采样次数

  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(2);
  }

  // 计算平均值
  float ax_avg = sum_ax / (float)N;
  float ay_avg = sum_ay / (float)N;
  float az_avg = sum_az / (float)N;
  float gx_avg = sum_gx / (float)N;
  float gy_avg = sum_gy / (float)N;
  float gz_avg = sum_gz / (float)N;

  // 计算偏移
  int16_t off_acc_x = -(int16_t)ax_avg;
  int16_t off_acc_y = -(int16_t)ay_avg;
  int16_t off_acc_z = -(int16_t)(az_avg - 16384);  // Z轴目标 1g

  int16_t off_gyro_x = -(int16_t)gx_avg;
  int16_t off_gyro_y = -(int16_t)gy_avg;
  int16_t off_gyro_z = -(int16_t)gz_avg;

  Serial.println("========== 校准完成 ==========");
  Serial.printf("mpu.setXAccelOffset(%d);\n", off_acc_x);
  Serial.printf("mpu.setYAccelOffset(%d);\n", off_acc_y);
  Serial.printf("mpu.setZAccelOffset(%d);\n", off_acc_z);
  Serial.println();
  Serial.printf("mpu.setXGyroOffset(%d);\n", off_gyro_x);
  Serial.printf("mpu.setYGyroOffset(%d);\n", off_gyro_y);
  Serial.printf("mpu.setZGyroOffset(%d);\n", off_gyro_z);
  Serial.println("==============================");

  return true;
}


void Peripherals::scanI2C()
{
    uint8_t targets[] = {0x68, 0x0D, 0x77, 0x40}; // MPU, HMC, OLED, PCA9685
    const char* names[] = {"MPU6050", "HMC5883L", "BMP180", "PCA9685"};

    Serial.println("--- I2C Scan ---");
    for(int i=0; i<4; i++) {
        Wire.beginTransmission(targets[i]);
        if (Wire.endTransmission() == 0) 
            Serial.printf("Found %s at 0x%02X\n", names[i], targets[i]);
    }

}


bool Peripherals::readMag() {
    // 这里应调用你项目中 HMC5883LDriver 的 update 逻辑
    // 假设你已经定义了 _mag 对象
    // return _mag.update(); 
    if (!i2c_active) return false;

    // 1. 从硬件获取最新磁场原始数据
    compass.read(); 

    /**
     * 2. 获取方位角 (Azimuth)
     * 该值范围为 0-359 度，0=北, 90=东, 180=南, 270=西
     * 库会根据你在 begin() 中设置的 setCalibration 和 setMagneticDeclination 自动计算
     */
    _mag_heading = compass.getAzimuth(); 

    return true;
}

bool Peripherals::readBMP() {
    // 这里应调用你项目中 BMP180Driver 的 update 逻辑
    // return _bmp.update();
    return true;
}

void Peripherals::fuseHeading() {
    float magHeading = (float)compass.getAzimuth(); // 0 到 359
    float imuYaw = angleZ(); // 获取 MPU6050 当前的 Yaw (-180 到 180)

    // 1. 将 IMU Yaw 转换到 0-360 空间，方便与磁力计对齐
    if (imuYaw < 0) imuYaw += 360;

    // 2. 初始对齐：开机时，让 IMU 的 Yaw 瞬间对准磁北
    if (_firstFusion) {
        _fusedYaw = magHeading;
        _yawOffset = magHeading - imuYaw;
        _firstFusion = false;
        return;
    }

    // 3. 计算偏差 (Handle wrap-around 359 -> 0)
    float error = magHeading - (imuYaw + _yawOffset);
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // 4. 互补滤波：权重系数建议在 0.001 到 0.01 之间
    // 这样既能消除漂移，又不会让磁力计的噪声引起机器人抖动
    float alpha = 0.005; 
    _yawOffset += error * alpha;

    // 5. 最终融合航向
    _fusedYaw = imuYaw + _yawOffset;

    // 规范化到 0-360
    while (_fusedYaw >= 360) _fusedYaw -= 360;
    while (_fusedYaw < 0) _fusedYaw += 360;
    float finalYaw = _fusedYaw;
    if (finalYaw > 180) finalYaw -= 360;
    ypr[0] = finalYaw * M_PI / 180;
}
