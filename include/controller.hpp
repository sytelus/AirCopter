#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "mux.hpp"
#include "param.hpp"
#include "estimator.hpp"
#include "mixer.hpp"
#include "CommonState.hpp"

class Controller {
public:
    void run_controller();
    void init(CommonState* _common_state, Mux* _mux, Mixer* _mixer, Estimator* _estimator, Params* _params);

private:
    typedef struct
    {
        Params::param_id_t kp_param_id;
        Params::param_id_t ki_param_id;
        Params::param_id_t kd_param_id;

        float* current_x;
        float* current_xdot;
        float* commanded_x;
        float* output;

        float max;
        float min;

        float integrator;
        float prev_time;
        float prev_x;
        float differentiator;
        float tau;
    } pid_t;

    pid_t pid_roll;
    pid_t pid_roll_rate;
    pid_t pid_pitch;
    pid_t pid_pitch_rate;
    pid_t pid_yaw_rate;
    pid_t pid_altitude;
    
    Estimator* estimator;
    CommonState* common_state;
    Mux* mux;
    Mixer* mixer;
    Params* params;

    void init_pid(pid_t* pid, Params::param_id_t kp_param_id, Params::param_id_t ki_param_id, Params::param_id_t kd_param_id, float* current_x, float* current_xdot, float* commanded_x, float* output, float max, float min);
    void run_pid(pid_t* pid, float dt);
};