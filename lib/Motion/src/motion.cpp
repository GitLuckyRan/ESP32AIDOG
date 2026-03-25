#include <motion.h>

void MotionService::begin() {
    body_state.updateFeet(KinConfig::default_feet_positions);
    currentMode = MOTION_STATE::STAND;
    setState(&standState);
}

void MotionService::setState(MotionState* newState) {
    Serial.println(">>> Changing State Now!");
    if (state) {
        state->end();
    }
    state = newState;
    if (state) {
        state->begin();
    }
}

void MotionService::handleWalkGait(bool trot) {
    if (trot)
        walkState.set_mode_trot();
    else
        walkState.set_mode_crawl();
}

void MotionService::handleMode(const String& cmd) {
    if (cmd.length() == 0) return;

    MOTION_STATE newMode;
    MotionState* newState;

    if (cmd.indexOf("WALK") >= 0) {
        newMode = MOTION_STATE::WALK;
        newState = &walkState;
    } else if (cmd.indexOf("STAND") >= 0) {
        newMode = MOTION_STATE::STAND;
        newState = &standState;
    } else if (cmd.indexOf("REST") >= 0 || cmd.indexOf("LIE") >= 0) {
        newMode = MOTION_STATE::REST;
        newState = &restState;
    } else {
        Serial.printf("Warning: Unknown command: %s\n", cmd.c_str());
        return;
    }

    if (newMode != currentMode) {
        currentMode = newMode;
        setState(newState);
        Serial.printf("New Mode Set: %s\n", cmd.c_str());
    }
}

bool MotionService::update(Peripherals* peripherals) {
    if (!state) return false;
    int64_t now = esp_timer_get_time();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    state->updateImuOffsets(peripherals->angleY(), -peripherals->angleX());
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
