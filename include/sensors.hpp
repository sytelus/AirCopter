#pragma once

#include <turbotrig/turbovec.h>
#include <stdint.h>
#include <stdbool.h>
#include "estimator.hpp"
#include "commonstate.hpp"
#include "param.hpp"
#include "board.hpp"
#include "commlink.hpp"

namespace rosflight {

class Sensors {
public:
    // function declarations
    void init(CommonState* _common_state, Board* _board, Estimator* _estimator, Params* _params, CommLink* _comm_link);
    bool update_sensors();

    bool start_imu_calibration(void);
    bool start_gyro_calibration(void);
    bool gyro_calibration_complete(void);

    void getImuMeasurements(vector_t& accel, vector_t& gyro, uint64_t& imu_time);

private:
    volatile uint8_t accel_status, gyro_status, temp_status;
    float accel_scale;
    float gyro_scale;
    bool calibrating_acc_flag;
    bool calibrating_gyro_flag;
    void calibrate_accel(void);
    void calibrate_gyro(void);
    void correct_imu(void);
    void imu_ISR(void);
    bool update_imu(void);

private:
    // IMU
    vector_t _accel;
    vector_t _gyro;
    float _imu_temperature;
    uint64_t _imu_time;
    bool new_imu_data = false;

    // Airspeed
    bool _diff_pressure_present = false;
    float _pitot_velocity, _pitot_diff_pressure, _pitot_temp;

    // Barometer
    bool _baro_present = false;
    float _baro_altitude;
    float _baro_pressure;
    float _baro_temperature;

    // Sonar
    bool _sonar_present = false;
    float _sonar_range;

    // Magnetometer
    bool _mag_present = false;
    vector_t _mag;

    // IMU stuff
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

    Estimator* estimator;
    CommonState* common_state;
    Params* params;
    Board* board;
    CommLink* comm_link;

    uint32_t last_time_look_for_disarmed_sensors = 0;
    uint32_t last_imu_update_ms = 0;
    uint16_t calib_gyro_count = 0;
    vector_t calib_gyro_sum = { 0.0f, 0.0f, 0.0f };
    uint16_t calib_accel_count = 0;
    vector_t calib_accel_sum = { 0.0f, 0.0f, 0.0f };
    static constexpr vector_t gravity = { 0.0f, 0.0f, 9.80665f };
    float acc_temp_sum = 0.0f;
};

} //namespace