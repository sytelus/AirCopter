#include <stdint.h>
#include "mixer.hpp"
#include "mode.hpp"
#include "rc.hpp"

namespace rosflight {

void Mixer::init(CommonState* _common_state, Board* _board, Params* _params)
{
    common_state = _common_state;
    params = _params;
    board = _board;

    // We need a better way to choosing the mixer
    mixer_to_use = *array_of_mixers[params->get_param_int(Params::PARAM_MIXER)];

    for (int8_t i = 0; i < 8; i++)
    {
        _outputs[i] = 0;
        prescaled_outputs[i] = 0;
        _GPIO_outputs[i] = 0;
        _GPIO_output_type[i] = NONE;
    }
    _command.F = 0;
    _command.x = 0;
    _command.y = 0;
    _command.z = 0;

    init_PWM();
}

void Mixer::init_PWM()
{
    bool useCPPM = false;
    if (params->get_param_int(Params::PARAM_RC_TYPE) == 1)
    {
        useCPPM = true;
    }
    int16_t motor_refresh_rate = params->get_param_int(Params::PARAM_MOTOR_PWM_SEND_RATE);
    int16_t off_pwm = 1000;
    board->pwmInit(useCPPM, false, false, motor_refresh_rate, off_pwm);
}


void Mixer::write_motor(uint8_t index, int32_t value)
{
    value += 1000;
    if (common_state->isArmed())
    {
        if (value > 2000)
        {
            value = 2000;
        } else if (value < params->get_param_int(Params::PARAM_MOTOR_IDLE_PWM) && params->get_param_int(Params::PARAM_SPIN_MOTORS_WHEN_ARMED))
        {
            value = params->get_param_int(Params::PARAM_MOTOR_IDLE_PWM);
        } else if (value < 1000)
        {
            value = 1000;
        }
    } else
    {
        value = 1000;
    }
    _outputs[index] = value;
    board->pwmWriteMotor(index, _outputs[index]);
}


void Mixer::write_servo(uint8_t index, int32_t value)
{
    if (value > 500)
    {
        value = 500;
    } else if (value < -500)
    {
        value = -500;
    }
    _outputs[index] = value + 1500;
    board->pwmWriteMotor(index, _outputs[index]);
}


void Mixer::mix_output()
{
    int32_t max_output = 0;

    // For now, we aren't supporting mixing with fixed wings.  This is a total hack, and should be re-thought
    if (params->get_param_int(Params::PARAM_FIXED_WING))
    {
        // AETR
        prescaled_outputs[0] = static_cast<int32_t>(_command.x);
        prescaled_outputs[1] = static_cast<int32_t>(_command.y);
        prescaled_outputs[2] = static_cast<int32_t>(_command.F * 1000); // Throttle comes in scaled from 0.0 to 1.0
        prescaled_outputs[3] = static_cast<int32_t>(_command.z);
    } else // For multirotors, domixing the same way (in fixed point for now);
    {

        for (int8_t i = 0; i < 8; i++)
        {
            if (mixer_to_use.output_type[i] != NONE)
            {
                // Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
                prescaled_outputs[i] = (int32_t)((_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
                    _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])*1000.0f);
                if (prescaled_outputs[i] > 1000 && prescaled_outputs[i] > max_output)
                {
                    max_output = prescaled_outputs[i];
                }
                // negative motor outputs are set to zero when writing to the motor,
                // but they have to be allowed here because the same logic is used for
                // servo commands, which may be negative
            }
        }

        // saturate outputs to maintain controllability even during aggressive maneuvers
        if (max_output > 1000)
        {
            int32_t scale_factor = 1000 * 1000 / max_output;
            for (int8_t i = 0; i < 8; i++)
            {
                if (mixer_to_use.output_type[i] == M)
                {
                    prescaled_outputs[i] = (prescaled_outputs[i])*scale_factor / 1000; // divide by scale factor
                }
            }
        }
    }

    // Reverse Fixedwing channels
    if (params->get_param_int(Params::PARAM_FIXED_WING))
    {
        prescaled_outputs[0] *= params->get_param_int(Params::PARAM_AILERON_REVERSE) ? -1 : 1;
        prescaled_outputs[1] *= params->get_param_int(Params::PARAM_ELEVATOR_REVERSE) ? -1 : 1;
        prescaled_outputs[3] *= params->get_param_int(Params::PARAM_RUDDER_REVERSE) ? -1 : 1;
    }

    // Add in GPIO inputs from Onboard Computer
    for (int8_t i = 0; i < 8; i++)
    {
        output_type_t output_type = mixer_to_use.output_type[i];
        if (output_type == NONE)
        {
            // Incorporate GPIO on not already reserved outputs
            prescaled_outputs[i] = _GPIO_outputs[i];
            output_type = _GPIO_output_type[i];
        }

        // Write output to motors
        if (output_type == S)
        {
            write_servo(i, prescaled_outputs[i]);
        } else if (output_type == M)
        {
            write_motor(i, prescaled_outputs[i]);
        }
    }
}


} //namespace