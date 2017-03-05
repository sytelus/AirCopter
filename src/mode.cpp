#include <stdbool.h>
#include "mux.hpp"
#include "sensors.hpp"
#include "board.hpp"
#include "mode.hpp"


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
    static bool started_gyro_calibration = false;
    if (!started_gyro_calibration && common_state->isDisarmed())
    {
        sensors->start_gyro_calibration();
        started_gyro_calibration = true;
        return false;
    } else if (sensors->gyro_calibration_complete())
    {
        started_gyro_calibration = false;
        common_state->setToArm();
        Board::setLed(0, true);
        return true;
    }
    return false;
}

void Mode::disarm(void)
{
    common_state->setToDisarm();
    Board::setLed(0, true);
}

/// TODO: Be able to tell if the RC has become disconnected during flight
bool Mode::check_failsafe(void)
{
    for (int8_t i = 0; i < params->get_param_int(Params::PARAM_RC_NUM_CHANNELS); i++)
    {
        if (Board::pwmRead(i) < 900 || Board::pwmRead(i) > 2100)
        {
            if (common_state->isArmed() || common_state->isDisarmed())
            {
                common_state->setArmedState(CommonState::FAILSAFE_DISARMED);
            }

            // blink LED
            static uint8_t count = 0;
            if (count > 25)
            {
                Board::toggleLed(1);
                count = 0;
            }
            count++;
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
    static uint64_t prev_time = 0;
    static uint32_t time_sticks_have_been_in_arming_position = 0;

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
                if (Board::pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) + params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && Board::pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) > (params->get_param_int(Params::PARAM_RC_Z_CENTER) + params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
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
                if (Board::pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) +
                    params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && Board::pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) < (params->get_param_int(Params::PARAM_RC_Z_CENTER) - params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
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
