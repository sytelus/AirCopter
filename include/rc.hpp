#ifndef RC_H_
#define RC_H_

#include "mux.hpp"
#include "param.hpp"
#include "board.hpp"
#include "commlink.hpp"

namespace rosflight {

class RC {
public:
    void init(CommonState* _common_state, Board* _board, Mux* _mux, Params* _params, CommLink* _comm_link);
    bool rc_switch(int16_t channel);
    bool receive_rc(uint64_t now);

private:
    void calibrate_rc();
    void convertPWMtoRad();

private:
    typedef struct
    {
        int16_t channel;
        int16_t direction;
    } rc_switch_t;

    typedef enum
    {
        PARALLEL_PWM,
        CPPM,
    } rc_type_t;


    bool _calibrate_rc;
    rc_switch_t switches[4];

    CommonState* common_state;
    Mux* mux;
    Params* params;
    Board* board;
    CommLink* comm_link;

    uint64_t last_rc_receive_time = 0; //receive_rc
    uint64_t time_of_last_stick_deviation = 0; //receive_rc
    int32_t calib_max[4] = {0, 0, 0, 0};
    int32_t calib_min[4] = {10000, 10000, 10000, 10000};
    int32_t calib_sum[4] = {0, 0, 0, 0};
    int32_t calib_count[4] = {0, 0, 0, 0};
};


} //namespace
#endif
