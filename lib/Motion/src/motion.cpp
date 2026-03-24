#include <motion.h>
#include "motion_states/walk_state.h"
#include "CamSerial.h"

void MotionService::begin() { body_state.updateFeet(KinConfig::default_feet_positions); 
    setState(&standState); }

void MotionService::handleAngles() {
    int  angles_count = 11111;  //待修复
    for (int i = 0; i < 12 && i < angles_count; i++) {
        angles[i] = angles[i];
    }
}

void MotionService::setState(MotionState* newState) {
    if (state) {
        state->end();
    }
    state = newState;
    if (state) {
        state->begin();
    }
}

void MotionService::handleInput() {
    // command.fromProto(data);
    // if (state) state->handleCommand(command);
}

void MotionService::handleWalkGait() {    
    MOTION_STATE mode = MOTION_STATE::REST;
    ESP_LOGI("MotionService", "Walk Gait %d", static_cast<int>(mode));
    if (mode ==  MOTION_STATE::REST)
        walkState.set_mode_trot();
    else
        walkState.set_mode_crawl();
}


//测试用，随机切换模式
void MotionService::handleMode() {
String Receive_cmd = CAM.SwitchCamMode(CAM.readCamSerial());
    static MOTION_STATE currentMode = MOTION_STATE::REST; // 初始设为 REST 比较安全

    // 只有当真正收到串口命令时，才进行状态切换
    if (Receive_cmd.length() > 0) {
        if (Receive_cmd.indexOf("WALK") >= 0) {
            if (currentMode != MOTION_STATE::WALK) { // 检查防止重复进入
                currentMode = MOTION_STATE::WALK;
                setState(&walkState);
            }
        } 
        else if (Receive_cmd.indexOf("STAND") >= 0) {
            if (currentMode != MOTION_STATE::STAND) {
                currentMode = MOTION_STATE::STAND;

                setState(&standState);
            }
        } 
        else if (Receive_cmd.indexOf("REST") >= 0) {
            if (currentMode != MOTION_STATE::REST) {
                currentMode = MOTION_STATE::REST;
                setState(&restState);
            }
        } 
        else {
            Serial.printf("Warning: Unknown command received: %s\n", Receive_cmd.c_str());
        }
        // 打印只在切换时执行，避免刷屏
        Serial.printf("New Mode Set: %s\n", 
            currentMode == MOTION_STATE::REST ? "REST" : 
            currentMode == MOTION_STATE::STAND ? "STAND" : "WALK");
    }
 

}



bool MotionService::update(Peripherals* peripherals) {
    handleMode();      //测试用，随机切换模式
    if (!state) return false;
    int64_t now = esp_timer_get_time();
    float dt = (now - lastUpdate) / 1000000.0f; // Convert microseconds to seconds
    lastUpdate = now;
    state->updateImuOffsets(peripherals->angleY(), peripherals->angleX());
    state->step(body_state, dt);
    kinematics.calculate_inverse_kinematics(body_state, new_angles);

    return update_angles(new_angles, angles);
}

bool MotionService::update_angles(float new_angles[12], float angles[12]) {
    bool updated = false;
    for (int i = 0; i < 12; i++) {
        const float new_angle = new_angles[i] * dir[i];
        if (!isEqual(new_angle, angles[i], 0.1)) {
            angles[i] = new_angle;
            updated = true;
        }
    }
    return updated;
}