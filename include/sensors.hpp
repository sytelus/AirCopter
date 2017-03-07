#pragma once

#include <turbotrig/turbovec.h>
#include <stdint.h>
#include <stdbool.h>
#include "estimator.hpp"
#include "commonstate.hpp"
#include "param.hpp"
#include "board.hpp"

class Sensors {
public:
    // function declarations
    void init_sensors(CommonState* _common_state, Board* _board, Estimator* _estimator, Params* _params);
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
};