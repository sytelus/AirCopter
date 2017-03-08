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


/************************************************** Implementation ***************************************************************/
void Mode::init(CommonState* _common_state, Sensors* _sensors, RC* _rc, Params* _params)
{
    params = _params;
    common_state = _common_state;
    sensors = _sensors;
    rc = _rc;

    common_state->setToDisarm();
}

bool Mode::arm(void)
{
    if (!started_gyro_calibration && common_state->isDisarmed())
    {
        sensors->start_gyro_calibration();
        started_gyro_calibration = true;
        return false;
    } else if (sensors->gyro_calibration_complete())
    {
        started_gyro_calibration = false;
        common_state->setToArm();
        board->setLed(0, true);
        return true;
    }
    return false;
}

void Mode::disarm(void)
{
    common_state->setToDisarm();
    board->setLed(0, true);
}

/// TODO: Be able to tell if the RC has become disconnected during flight
bool Mode::check_failsafe(void)
{
    for (int8_t i = 0; i < params->get_param_int(Params::PARAM_RC_NUM_CHANNELS); i++)
    {
        if (board->pwmRead(i) < 900 || board->pwmRead(i) > 2100)
        {
            if (common_state->isArmed() || common_state->isDisarmed())
            {
                common_state->setArmedState(CommonState::FAILSAFE_DISARMED);
            }

            // blink LED
            if (blink_count > 25)
            {
                board->toggleLed(1);
                blink_count = 0;
            }
            blink_count++;
            return true;
        }
    }

    // we got a valid RC measurement for all channels
    if (common_state->getArmedState() == CommonState::FAILSAFE_ARMED || common_state->getArmedState() == CommonState::FAILSAFE_DISARMED)
    {
        // return to appropriate mode
        common_state->setArmedState(
            (common_state->getArmedState() == CommonState::FAILSAFE_ARMED) ? CommonState::ARMED :CommonState::DISARMED
        );
    }
    return false;
}


bool Mode::check_mode(uint64_t now)
{
    // see it has been at least 20 ms
    uint32_t dt = static_cast<uint32_t>(now - prev_time);
    if (dt < 20000)
    {
        return false;
    }

    // if it has, then do stuff
    prev_time = now;

    // check for failsafe mode
    if (check_failsafe())
    {
        return true;
    } else
    {
        // check for arming switch
        if (params->get_param_int(Params::PARAM_ARM_STICKS))
        {
            if (common_state->getArmedState() == CommonState::DISARMED)
            {
                // if left stick is down and to the right
                if (board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) + params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) > (params->get_param_int(Params::PARAM_RC_Z_CENTER) + params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
                    - params->get_param_int(Params::PARAM_ARM_THRESHOLD))
                {
                    time_sticks_have_been_in_arming_position += dt;
                } else
                {
                    time_sticks_have_been_in_arming_position = 0;
                }
                if (time_sticks_have_been_in_arming_position > 500000)
                {
                    if (arm())
                        time_sticks_have_been_in_arming_position = 0;
                }
            } else // _armed_state is ARMED
            {
                // if left stick is down and to the left
                if (board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) +
                    params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) < (params->get_param_int(Params::PARAM_RC_Z_CENTER) - params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
                    + params->get_param_int(Params::PARAM_ARM_THRESHOLD))
                {
                    time_sticks_have_been_in_arming_position += dt;
                } else
                {
                    time_sticks_have_been_in_arming_position = 0;
                }
                if (time_sticks_have_been_in_arming_position > 500000)
                {
                    disarm();
                    time_sticks_have_been_in_arming_position = 0;
                }
            }
        } else
        {
            if (rc->rc_switch(params->get_param_int(Params::PARAM_ARM_CHANNEL)))
            {
                if (common_state->getArmedState() == CommonState::DISARMED)
                    arm();
            } else
            {
                disarm();
            }
        }
    }
    return true;
}



} //namespace