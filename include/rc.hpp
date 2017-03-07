#ifndef RC_H_
#define RC_H_

#include "mux.hpp"
#include "param.hpp"
#include "board.hpp"

class RC {
public:
    void init(CommonState* _common_state, Board* _board, Mux* _mux, Params* _params);
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
};

#endif
