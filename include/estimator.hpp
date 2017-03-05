#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <turbotrig/turbotrig.h>
#include <turbotrig/turbovec.h>
#include "param.hpp"

class Estimator {
public:
    typedef struct
    {
        quaternion_t q;
        vector_t euler;
        vector_t omega;

        //  float p;
        //  float q;
        //  float r;

        //  float phi;
        //  float theta;
        //  float psi;

        float altitude;
        uint64_t now_us;

    } state_t;

public:
    void init_estimator(Params* _params, bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
    void reset_state();
    void reset_adaptive_bias();
    void run_estimator(const vector_t& accel, const vector_t& gyro, const uint64_t& imu_time);
    state_t& getState() { return _current_state; };

private:
    void run_LPF(const vector_t& accel, const vector_t& gyro);


private:
    state_t _current_state;
    Params* params;
    vector_t _adaptive_gyro_bias;

    vector_t w1;
    vector_t w2;
    vector_t wbar;
    vector_t wfinal;
    vector_t w_acc;
    static constexpr vector_t g = {0.0f, 0.0f, -1.0f};
    vector_t b;
    quaternion_t q_tilde;
    quaternion_t q_hat;
    uint64_t last_time;

    bool mat_exp;
    bool quad_int;
    bool use_acc;

    vector_t _accel_LPF;
    vector_t _gyro_LPF;
};