#ifndef ServoController_h
#define ServoController_h
#include <Adafruit_PWMServoDriver.h>

#include <math_utils.h>
#include <algorithm>
#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif



#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 27000000
#endif


#define PCA9685_ADDR 0x40
using ::clamp;

typedef struct _api_Servo {
    float center_pwm;
    float direction;
    float center_angle;
    float conversion;
    char name[16];
} api_Servo;



typedef struct _api_ServoSettings {
    uint_least16_t servos_count;
    api_Servo servos[12]; /* max 12 servos */
} api_ServoSettings;


enum class SERVO_CONTROL_STATE { DEACTIVATED, PWM, ANGLE };


Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDR);
using ServoSettings = api_ServoSettings;


// const api_Servo defaults[12] = {
//     {302, -1, 0, 2.33f, "Servo1"}, {322, 1, -45, 2.48f, "Servo2"},  {302, 1, 90, 2.5f, "Servo3"},
//     {302, -1, 0, 2.33f, "Servo4"}, {282, -1, 45, 2.48f, "Servo5"},  {282, -1, -90, 2.33f, "Servo6"},
//     {302, 1, 0, 2.48f, "Servo7"},  {322, 1, -45, 2.48f, "Servo8"},  {322, 1, 90, 2.48f, "Servo9"},
//     {302, 1, 0, 2.48f, "Servo10"}, {302, -1, 45, 2.48f, "Servo11"}, {322, -1, -90, 2.48f, "Servo12"}};
const api_Servo defaults[12] = {
    {282, -1, 0, 2.33f, "Servo1"}, {322, 1, -45, 2.48f, "Servo2"},  {302, 1, 90, 2.48f, "Servo3"},
    {272, -1, 0, 2.33f, "Servo4"}, {282, -1, 45, 2.48f, "Servo5"},  {302, -1, -90, 2.48f, "Servo6"},
    {302, 1, 0, 2.48f, "Servo7"},  {302, 1, -45, 2.48f, "Servo8"},  {302, 1, 90, 2.48f, "Servo9"},
    {302, 1, 0, 2.48f, "Servo10"}, {272, -1, 45, 2.48f, "Servo11"}, {302, -1, -90, 2.48f, "Servo12"}};

inline ServoSettings ServoSettings_defaults() {
    ServoSettings settings = {};
    settings.servos_count = 12;
    for (int i = 0; i < 12; i++) {
        settings.servos[i] = defaults[i];
    }
    return settings;
}



class ServoController {
  public:
    ServoController() = default;
    void begin() {
        initializePCA();
    }



    void pcaWrite(int index, int value) {
        if (value < 0 || value > 4096) {
            ESP_LOGE("Peripherals", "Invalid PWM value %d for %d :: Valid range 0-4096", value, index);
            return;
        }
        pca9685.setPWM(index, 0, value);
    }

    void activate() {
        if (is_active) return;
        control_state = SERVO_CONTROL_STATE::ANGLE;
        is_active = true;
        pca9685.wakeup();
    }

    void deactivate() {
        if (!is_active) return;
        is_active = false;
        control_state = SERVO_CONTROL_STATE::DEACTIVATED;
        pca9685.sleep();
    }

    void setServoPWM(int32_t servo_id, uint32_t pwm) {
        control_state = SERVO_CONTROL_STATE::PWM;
        if (servo_id < 0) {
            uint16_t pwms[12];
            std::fill_n(pwms, 12, static_cast<uint16_t>(pwm));
            for (uint8_t i = 0; i < 12; i++) {
                pca9685.setPWM(i, 0, pwm);
              }
        } else {
            pca9685.setPWM(servo_id, 0, pwm);
        }
    }

    void updateActiveState() { is_active ? activate() : deactivate(); }

    void setMode(SERVO_CONTROL_STATE newMode) { control_state = newMode; }

    void setAngles(float new_angles[12]) {
        for (int i = 0; i < 12; i++) {
            target_angles[i] = new_angles[i];
            // Serial.printf("Target angle for servo %d set to %.2f\n", i + 1, new_angles[i]);
        }
    }
    float smoothing_factor = 0.5f;
    void setSmoothing(float factor) { smoothing_factor = factor; }

    void calculatePWM() {
        uint16_t pwms[12];
       
        for (int i = 0; i < 12; i++) {

            angles[i] = lerp(angles[i], target_angles[i], smoothing_factor);
            auto &servo = defaults[i];
            float angle = servo.direction * angles[i] + servo.center_angle;
            uint16_t pwm = angle * servo.conversion + servo.center_pwm;
            pwms[i] = pwm = clamp(pwm, 120, 500);
            // Serial.printf("Servo %d -> Angle: %6.2f°, PWM: %4d\n", i + 1, angles[i], pwm);
        }

        for (uint8_t i = 0; i < 12; i++) {
    // 忽略所有计算出的动作，直接发送预设的中位脉冲值
            pca9685.setPWM(i, 0, pwms[i]);
        }


    }
    void update() {
       
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

  private:
    void initializePCA() {
        pca9685.begin();
        pca9685.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        pca9685.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        pca9685.sleep();
    }


    SERVO_CONTROL_STATE control_state = SERVO_CONTROL_STATE::DEACTIVATED;

    bool is_active {false};
    // float angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
    // float target_angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
    float angles[12] = {0};
    float target_angles[12] = {0};
};

#endif
