#pragma once


namespace rosflight {

class CommonState {
public:
    typedef enum
    {
        ARMED,
        DISARMED,
        FAILSAFE_ARMED,
        FAILSAFE_DISARMED
    } armed_state_t;

    armed_state_t get_armed_state() {
        return _armed_state;
    }
    void setArmedState(armed_state_t state) {
        _armed_state = state;
    }

    bool is_armed()
    {
        return  _armed_state == armed_state_t::ARMED;
    }
    bool is_disarmed()
    {
        return  _armed_state == armed_state_t::DISARMED;
    }
    void set_disarm()
    {
        _armed_state = armed_state_t::DISARMED;
    }
    void set_arm()
    {
        _armed_state = armed_state_t::ARMED;
    }

private:
    armed_state_t _armed_state;
};


} //namespace