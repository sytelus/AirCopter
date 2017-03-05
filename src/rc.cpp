#include <stdint.h>
#include <stdbool.h>
#include <algorithm>
#include "rc.hpp"
#include "board.hpp"
#include "api.hpp"

void RC::init(CommonState* _common_state, Mux* _mux, Params* _params)
{
  common_state = _common_state;
  mux = _mux;
  params = _params;

  _calibrate_rc = false;

  Mux::control_t& rc_control = mux->getRCControl();
  rc_control.x.type = Mux::ANGLE;
  rc_control.y.type = Mux::ANGLE;
  rc_control.z.type = Mux::RATE;
  rc_control.F.type = Mux::THROTTLE;

  rc_control.x.value = 0;
  rc_control.y.value = 0;
  rc_control.z.value = 0;
  rc_control.F.value = 0;

  Mux::control_t& offboard_control = mux->getOffboardControl();
  offboard_control.x.active = false;
  offboard_control.y.active = false;
  offboard_control.z.active = false;
  offboard_control.F.active = false;

  switches[0].channel = 4;
  switches[0].direction = params->get_param_int(Params::PARAM_RC_SWITCH_5_DIRECTION);
  switches[1].channel = 5;
  switches[1].direction = params->get_param_int(Params::PARAM_RC_SWITCH_6_DIRECTION);
  switches[2].channel = 6;
  switches[2].direction = params->get_param_int(Params::PARAM_RC_SWITCH_7_DIRECTION);
  switches[3].channel = 7;
  switches[3].direction = params->get_param_int(Params::PARAM_RC_SWITCH_8_DIRECTION);
}

bool RC::rc_switch(int16_t channel)
{
  if(channel < 4 || channel > 8)
  {
    return false;
  }
  if(switches[channel - 4].direction < 0)
  {
    return Board::pwmRead(channel) < 1500;
  }
  else
  {
    return Board::pwmRead(channel) > 1500;
  }
}

void RC::convertPWMtoRad()
{
    Mux::control_t& rc_control = mux->getRCControl();

  // Get Roll control command out of RC
  if (rc_control.x.type == Mux::ANGLE)
  {
    rc_control.x.value = (float)((Board::pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - 1500)
                           *2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLL))/(float)params->get_param_int(Params::PARAM_RC_X_RANGE);
  }
  else if (rc_control.x.type == Mux::RATE)
  {
    rc_control.x.value = (float)((Board::pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - 1500)
                            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLLRATE))/(float)params->get_param_int(Params::PARAM_RC_X_RANGE);
  }
  else if (rc_control.x.type == Mux::PASSTHROUGH)
  {
    rc_control.x.value = static_cast<float>(
            Board::pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - params->get_param_int(Params::PARAM_RC_X_CENTER)
        );
  }

  // Get Pitch control command out of RC
  if (rc_control.y.type == Mux::ANGLE)
  {
    rc_control.y.value = ((Board::pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500)
                            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCH))/(float)params->get_param_int(Params::PARAM_RC_Y_RANGE);
  }
  else if (rc_control.y.type == Mux::RATE)
  {
    rc_control.y.value = (float)((Board::pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500)
                            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCHRATE))/(float)params->get_param_int(Params::PARAM_RC_Y_RANGE);
  }
  else if (rc_control.y.type == Mux::PASSTHROUGH)
  {
    rc_control.y.value = static_cast<float>(
            Board::pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500
        );
  }

  // Get the Yaw control command type out of RC
  if (rc_control.z.type == Mux::RATE)
  {
    rc_control.z.value = ((Board::pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - 1500)
                           *2.0f*params->get_param_float(Params::PARAM_RC_MAX_YAWRATE))/(float)params->get_param_int(Params::PARAM_RC_Z_RANGE);
  }
  else if (rc_control.z.type == Mux::PASSTHROUGH)
  {
    rc_control.z.value = static_cast<float>(Board::pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - 1500);
  }

  // Finally, the Mux::THROTTLE command
  rc_control.F.value = (float)((Board::pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) - params->get_param_int(Params::PARAM_RC_F_BOTTOM)))
                        / (float)params->get_param_int(Params::PARAM_RC_F_RANGE);
}


bool RC::receive_rc(uint64_t now)
{
  if(_calibrate_rc)
  {
    calibrate_rc();
  }
  // if it has been more than 20ms then look for new RC values and parse them
  static uint64_t last_rc_receive_time = 0;
  static uint64_t time_of_last_stick_deviation = 0;

  if (now - last_rc_receive_time < 20000)
  {
    return false;
  }
  last_rc_receive_time = now;
  // Get timestamp for deadband control lag

  Mux::control_t& rc_control = mux->getRCControl();

  // Figure out the desired control type from the switches and params
  if (params->get_param_int(Params::PARAM_FIXED_WING))
  {
    // for using fixedwings
    rc_control.x.type = rc_control.y.type = rc_control.z.type = Mux::PASSTHROUGH;
    rc_control.F.type = Mux::THROTTLE;
  }
  else
  {
    rc_control.x.type = rc_control.y.type = rc_switch(params->get_param_int(Params::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? Mux::ANGLE : Mux::RATE;
    rc_control.z.type = Mux::RATE;
    rc_control.F.type = rc_switch(params->get_param_int(Params::PARAM_RC_F_CONTROL_TYPE_CHANNEL)) ? Mux::ALTITUDE : Mux::THROTTLE;
  }

  // Interpret PWM Values from RC
  convertPWMtoRad();

  // Set flags for attitude channels
  if (rc_switch(params->get_param_int(Params::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL))
      || now - time_of_last_stick_deviation < (uint32_t)(params->get_param_int(Params::PARAM_OVERRIDE_LAG_TIME))*1000)
  {
    // Pilot is in full control
    rc_control.x.active = true;
    rc_control.y.active = true;
    rc_control.z.active = true;
  }
  else
  {
    // Check for stick deviation - if so, then the channel is active
    rc_control.x.active = rc_control.y.active  = rc_control.z.active =
                             abs(Board::pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - params->get_param_int(Params::PARAM_RC_X_CENTER)) >
                             params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION)
                             || abs(Board::pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - params->get_param_int(Params::PARAM_RC_Y_CENTER)) >
                             params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION)
                             || abs(Board::pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - params->get_param_int(Params::PARAM_RC_Z_CENTER)) >
                             params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION);
    if (rc_control.x.active)
    {
      // reset override lag
      time_of_last_stick_deviation = now;
    }
  }


  // Set flags for Mux::THROTTLE channel
  if (rc_switch(params->get_param_int(Params::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
  {
    // RC Pilot is in full control
    rc_control.F.active = true;
  }
  else
  {
    // Onboard Control - min Mux::THROTTLE Checking will be done in mux and in the controller.
    rc_control.F.active = false;
  }

  mux->setNewCommand(true);
  return true;
}

void RC::calibrate_rc()
{
  if(common_state->isArmed())
  {
    Api::logMessage("Cannot calibrate RC when FCU is armed", 5);
  }
  else
  {
    // Calibrate Extents of RC Transmitter
    Api::logMessage("Calibrating RC, move sticks to full extents", 1);
    Api::logMessage("in the next 10s", 1);
    uint64_t now = Board::micros();
    static int32_t max[4] = {0, 0, 0, 0};
    static int32_t min[4] = {10000, 10000, 10000, 10000};
    while(Board::micros() - now < 1e7)
    {
      for(int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)Board::pwmRead(i);
        if(read_value > max[i])
        {
          max[i] = read_value;
        }
        if(read_value < min[i])
        {
          min[i] = read_value;
        }
      }
      Board::delay(10);
    }
    params->set_param_int(Params::PARAM_RC_X_RANGE, max[params->get_param_int(Params::PARAM_RC_X_CHANNEL)] - min[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_Y_RANGE, max[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)] - min[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_Z_RANGE, max[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)] - min[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_F_RANGE, max[params->get_param_int(Params::PARAM_RC_F_CHANNEL)] - min[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]);

    // Calibrate Trimmed Centers
    Api::logMessage("Calibrating RC, leave sticks at center", 1);
    Api::logMessage("and Mux::THROTTLE low for next 10 seconds", 1);
    Board::delay(5000);
    now = Board::micros();
    static int32_t sum[4] = {0, 0, 0, 0};
    static int32_t count[4] = {0, 0, 0, 0};

    while(Board::micros() - now < 5e6)
    {
      for(int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)Board::pwmRead(i);
        sum[i] = sum[i] + read_value;
        count[i] = count[i] + 1;
      }
      Board::delay(20); // RC is updated at 50 Hz
    }

    params->set_param_int(Params::PARAM_RC_X_CENTER, sum[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]/count[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_Y_CENTER, sum[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]/count[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_Z_CENTER, sum[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]/count[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]);
    params->set_param_int(Params::PARAM_RC_F_BOTTOM, sum[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]/count[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]);
  }

  // calculate Trim values (in terms of SI units)
  if(rc_switch(params->get_param_int(Params::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)))
  {
    // in angle mode
    params->set_param_float(Params::PARAM_ROLL_ANGLE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_X_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLL)
                          /(float)params->get_param_int(Params::PARAM_RC_X_RANGE));
    params->set_param_float(Params::PARAM_PITCH_ANGLE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Y_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCH)
                          /(float)params->get_param_int(Params::PARAM_RC_Y_RANGE));
  }
  else
  {
    // in rate mode
    params->set_param_float(Params::PARAM_ROLL_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_X_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLLRATE)
                          /(float)params->get_param_int(Params::PARAM_RC_X_RANGE));
    params->set_param_float(Params::PARAM_PITCH_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Y_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCHRATE)
                          /(float)params->get_param_int(Params::PARAM_RC_Y_RANGE));
  }
  params->set_param_float(Params::PARAM_YAW_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Z_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_YAWRATE)
                        /(float)params->get_param_int(Params::PARAM_RC_Z_RANGE));

  params->write_params();

  Api::logMessage("Completed RC calibration", 0);
  _calibrate_rc = false;
}

