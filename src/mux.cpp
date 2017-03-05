#include <stdbool.h>
#include "board.hpp"
#include "rc.hpp"
#include "mux.hpp"
#include "mode.hpp"

void Mux::init(CommonState* _common_state, Params* _params)
{
    params = _params;
    common_state = _common_state;
}

bool Mux::mux_inputs()
{
  if (!_new_command)
  {
    // we haven't received any new commands, so we shouldn't do anything
    return false;
  }
  // otherwise combine the new commands

  if(common_state->getArmedState() == CommonState::FAILSAFE_ARMED || common_state->getArmedState() == CommonState::FAILSAFE_DISARMED)
  {
    _combined_control = _failsafe_control;  
  }
  else
  {
    if (_rc_control.x.active)
    {
      _combined_control.x = _rc_control.x;
    }
    else if (_offboard_control.x.active)
    {
      _combined_control.x = _offboard_control.x;
    }
    else
    {
      // default to taking RC if neither is publishing
      _combined_control.x = _rc_control.x;
      _combined_control.x.active = true;
    }


    if (_rc_control.y.active)
    {
      _combined_control.y = _rc_control.y;
    }
    else if (_offboard_control.y.active)
    {
      _combined_control.y = _offboard_control.y;
    }
    else
    {
      // default to taking RC if neither is publishing
      _combined_control.y = _rc_control.y;
      _combined_control.y.active = true;
    }


    if (_rc_control.z.active)
    {
      _combined_control.z = _rc_control.z;
    }
    else if (_offboard_control.z.active)
    {
      _combined_control.z = _offboard_control.z;
    }
    else
    {
      _combined_control.z = _rc_control.z;
      _combined_control.z.active = true;
    }

    if (params->get_param_int(Params::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
    {
      if (_offboard_control.F.active)
      {
        if (_rc_control.F.type == THROTTLE && _offboard_control.F.type == THROTTLE)
        {
          _combined_control.F.value = (_rc_control.F.value > _offboard_control.F.value) ?
                                      _offboard_control.F.value : _rc_control.F.value;
          _combined_control.F.type = THROTTLE;
          _combined_control.F.active = true;
        }
        else
        {
          // I'm still not quite sure how to handle the mixed altitude/throttle cases
          // for now, just pass the rc along.  I expect that what we really need to do
          // is run the altitude controller here so we can compare throttle to throttle
          _combined_control.F = _rc_control.F;
        }
      }
    }
    else // no min throttle check
    {
      if (_rc_control.F.active)
      {
        _combined_control.F = _rc_control.F;
      }
      else if (_offboard_control.F.active)
      {
        _combined_control.F = _offboard_control.F;
      }
      else
      {
        _combined_control.F = _rc_control.F;
        _combined_control.F.active = true;
      }
    }

    // Light to indicate override
    if (_rc_control.x.active || _rc_control.y.active || _rc_control.z.active || _rc_control.F.active)
    {
      Board::setLed(0, true);
    }
    else
    {
      Board::setLed(0, false);
    }
  }

  // reset the new command flag
  _new_command = false;
  return true;
}

