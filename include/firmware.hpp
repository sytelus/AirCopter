#pragma once

#include "estimator.hpp"
#include "mode.hpp"
#include "param.hpp"
#include "sensors.hpp"
#include "controller.hpp"
#include "mixer.hpp"
#include "rc.hpp"
#include "board.hpp"
#include "commlink.hpp"
#include "commonstate.hpp"


namespace rosflight {

class Firmware {
public:
    Firmware(Board* _board, CommLink* _comm_link);

    void setup();
    void loop();

private:
    //params and shared state
    Params params;
    CommonState common_state;

    //objects we use
    Board* board;
    CommLink* comm_link;
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

} //namespace