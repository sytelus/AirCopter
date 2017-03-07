#pragma once

class CommonState {
public:
    typedef enum
    {
        ARMED,
        DISARMED,
        FAILSAFE_ARMED,
        FAILSAFE_DISARMED
    } armed_state_t;

    armed_state_t getArmedState() {
        return _armed_state;
    }
    void setArmedState(armed_state_t state) {
        _armed_state = state;
    }

    bool isArmed()
    {
        return  _armed_state == armed_state_t::ARMED;
    }
    bool isDisarmed()
    {
        return  _armed_state == armed_state_t::DISARMED;
    }
    void setToDisarm()
    {
        _armed_state = armed_state_t::DISARMED;
    }
    void setToArm()
    {
        _armed_state = armed_state_t::ARMED;
    }

private:
    armed_state_t _armed_state;
};