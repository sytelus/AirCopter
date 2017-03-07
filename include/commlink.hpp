#pragma once

#include <cstdint>


namespace rosflight {

class CommLink {
public:
    virtual void init() = 0;
    virtual void send(uint64_t cur_micros) = 0;
    virtual void receive() = 0;
    virtual void setSysID(int32_t sys_id) = 0;
    virtual void setStreamingRate(uint16_t param_id, int32_t rate) = 0;
    virtual void notifyParamChange(uint16_t param_id, int32_t value) = 0;
    virtual void logMessage(const char* message, uint8_t error_level) = 0;
    virtual void notifyControllerUpdated() = 0;
};


} //namespace