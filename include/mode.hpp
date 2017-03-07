#pragma once

#include <stdint.h>
#include "sensors.hpp"
#include "rc.hpp"
#include "commonstate.hpp"
#include "param.hpp"


namespace rosflight {

class Mode {
public:
    typedef enum
    {
        INVALID_CONTROL_MODE,
        INVALID_ARMED_STATE,
    } error_state_t;

    void init(CommonState* _common_state, Sensors* _sensors, RC* _rc, Params* _params);
    bool check_mode(uint64_t now);

private:
    bool arm(void);
    void disarm(void);
    bool check_failsafe(void);

private:
    error_state_t _error_state;
    CommonState* common_state;
    Sensors* sensors;
    RC* rc;
    Params* params;
    Board* board;

    bool started_gyro_calibration = false; //arm
    uint8_t blink_count = 0; //check_failsafe
    uint64_t prev_time = 0; //check_mode
    uint32_t time_sticks_have_been_in_arming_position = 0; //check_mode
};


} //namespace