#pragma once

#include "estimator.hpp"
#include "mode.hpp"
#include "param.hpp"
#include "sensors.hpp"
#include "controller.hpp"
#include "mixer.hpp"
#include "rc.hpp"
#include "board.hpp"
#include "api.hpp"
#include "commonstate.hpp"


class Firmware {
public:
    Firmware(Board* _board);

    void setup();
    void loop();

private:
    //params and shared state
    Params params;
    CommonState common_state;

    //objects we use
    Board* board;
    Estimator estimator;
    Sensors sensors;
    Mux mux;
    Mixer mixer;
    Controller controller;
    RC rc;
    Mode mode;
    
    //variables to real IMU
    vector_t accel, gyro;
    uint64_t imu_time;
};