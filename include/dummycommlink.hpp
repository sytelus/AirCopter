#pragma once

#include "commlink.hpp"


namespace rosflight {

class DummyCommLink : public CommLink {
public:
    virtual void init() override {}
    virtual void send(uint64_t cur_micros) override {}
    virtual void receive() override {}
    virtual void setSysID(int32_t sys_id) override {}
    virtual void setStreamingRate(uint16_t param_id, int32_t rate) override {}
    virtual void notifyParamChange(uint16_t param_id, int32_t value) override {}
    virtual void logMessage(const char* message, uint8_t error_level) override {}
};


} //namespace