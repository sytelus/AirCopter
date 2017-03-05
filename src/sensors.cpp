
#include <stdbool.h>
#include <stdint.h>
#include "sensors.hpp"
#include "estimator.hpp"
#include "mode.hpp"
#include "board.hpp"
#include "turbotrig/turbovec.h"
#include "api.hpp"



void Sensors::init_sensors(CommonState* _common_state, Estimator* _estimator, Params* _params)
{
    common_state = _common_state;
    estimator = _estimator;
    params = _params;

    uint16_t acc1G;
    Board::initSensors(acc1G, gyro_scale, params->get_param_int(Params::PARAM_BOARD_REVISION), std::bind(&Sensors::imu_ISR, this));
    accel_scale = 9.80665f / acc1G * params->get_param_float(Params::PARAM_ACCEL_SCALE);
}

void Sensors::getImuMeasurements(vector_t& accel, vector_t& gyro, uint64_t& imu_time)
{
    accel = _accel;
    gyro = _gyro;
    imu_time = _imu_time;
}

bool Sensors::update_sensors()
{
    // Look for disabled sensors while disarmed (poll every 10 seconds)
    // These sensors need power to respond, so they might not have been
    // detected on startup, but will be detected whenever power is applied
    // to the 5V rail.
    static uint32_t last_time_look_for_disarmed_sensors = 0;
    if (common_state->isDisarmed() && (!_sonar_present))//|| !_diff_pressure_present))
    {
        uint32_t now = Board::millis();
        if (now > (last_time_look_for_disarmed_sensors + 500))
        {
            last_time_look_for_disarmed_sensors = now;
            if (!_sonar_present)
            {
                _sonar_present = Board::isSensorPresent(Board::SensorType::Sonar);
                if (_sonar_present)
                {
                    Api::logMessage("FOUND SONAR", 0);
                }
            }
            if (!_diff_pressure_present)
            {
                _diff_pressure_present = Board::isSensorPresent(Board::SensorType::DiffPressure);
                if (_diff_pressure_present)
                {
                    Api::logMessage("FOUND DIFF PRESS", 0);
                }
            }
        }
    }

    if (_baro_present)
    {
        Board::readBaro(_baro_altitude, _baro_pressure, _baro_temperature);
    }

    if (_diff_pressure_present)
    {
        Board::readDiffPressure(_pitot_diff_pressure, _pitot_temp, _pitot_velocity);
    }

    if (_sonar_present)
    {
        _sonar_range = Board::readSonar();
    }

    if (_mag_present)
    {
        int16_t raw_mag[3] = { 0,0,0 };
        Board::readMag(raw_mag);
        _mag.x = (float)raw_mag[0];
        _mag.y = (float)raw_mag[1];
        _mag.z = (float)raw_mag[2];
    }

    // Return whether or not we got new IMU data
    return update_imu();
}


bool Sensors::start_imu_calibration(void)
{
    start_gyro_calibration();

    calibrating_acc_flag = true;
    params->set_param_float(Params::PARAM_ACC_X_BIAS, 0.0);
    params->set_param_float(Params::PARAM_ACC_Y_BIAS, 0.0);
    params->set_param_float(Params::PARAM_ACC_Z_BIAS, 0.0);
    return true;
}

bool Sensors::start_gyro_calibration(void)
{
    calibrating_gyro_flag = true;
    params->set_param_float(Params::PARAM_GYRO_X_BIAS, 0.0);
    params->set_param_float(Params::PARAM_GYRO_Y_BIAS, 0.0);
    params->set_param_float(Params::PARAM_GYRO_Z_BIAS, 0.0);
    return true;
}

bool Sensors::gyro_calibration_complete(void)
{
    return !calibrating_gyro_flag;
}


void Sensors::imu_ISR(void)
{
    _imu_time = Board::micros();
    new_imu_data = true;
}


bool Sensors::update_imu(void)
{
    static uint32_t last_imu_update_ms = 0;

    if (new_imu_data)
    {
        last_imu_update_ms = Board::millis();
        Board::readAccel(accel_raw);
        Board::readGyro(gyro_raw);
        Board::readTemperature(temp_raw);
        new_imu_data = false;

        // convert temperature SI units (degC, m/s^2, rad/s)
        _imu_temperature = temp_raw / 340.0f + 36.53f;

        // convert to NED and SI units
        _accel.x = accel_raw[0] * accel_scale;
        _accel.y = -accel_raw[1] * accel_scale;
        _accel.z = -accel_raw[2] * accel_scale;

        _gyro.x = gyro_raw[0] * gyro_scale;
        _gyro.y = -gyro_raw[1] * gyro_scale;
        _gyro.z = -gyro_raw[2] * gyro_scale;

        if (calibrating_acc_flag == true)
            calibrate_accel();
        if (calibrating_gyro_flag)
            calibrate_gyro();

        correct_imu();
        return true;
    } else
    {
        // if we have lost 1000 IMU messages something is wrong
        if (Board::millis() > last_imu_update_ms + 1000)
        {
            // change board revision and reset IMU
            last_imu_update_ms = Board::millis();
            params->set_param_int(Params::PARAM_BOARD_REVISION, (params->get_param_int(Params::PARAM_BOARD_REVISION) >= 4) ? 2 : 5);
            uint16_t acc1G;
            Board::initImu(acc1G, gyro_scale, params->get_param_int(Params::PARAM_BOARD_REVISION));
            accel_scale = 9.80665f / acc1G * params->get_param_float(Params::PARAM_ACCEL_SCALE);
        }
        return false;
    }
}


void Sensors::calibrate_gyro()
{
    static uint16_t count = 0;
    static vector_t gyro_sum = { 0.0f, 0.0f, 0.0f };
    gyro_sum = vector_add(gyro_sum, _gyro);
    count++;

    if (count > 100)
    {
        // Gyros are simple.  Just find the average during the calibration
        vector_t gyro_bias = scalar_multiply(1.0f / (float)count, gyro_sum);

        if (sqrd_norm(gyro_bias) < 1.0)
        {
            params->set_param_float(Params::PARAM_GYRO_X_BIAS, gyro_bias.x);
            params->set_param_float(Params::PARAM_GYRO_Y_BIAS, gyro_bias.y);
            params->set_param_float(Params::PARAM_GYRO_Z_BIAS, gyro_bias.z);

            // Tell the estimator to reset it's bias estimate, because it should be zero now
            estimator->reset_adaptive_bias();
        } else
        {
            Api::logMessage("Too much movement for gyro cal", 3);
        }

        // reset calibration in case we do it again
        calibrating_gyro_flag = false;
        count = 0;
        gyro_sum.x = 0.0f;
        gyro_sum.y = 0.0f;
        gyro_sum.z = 0.0f;
    }
}


void Sensors::calibrate_accel(void)
{
    static uint16_t count = 0;
    static vector_t acc_sum = { 0.0f, 0.0f, 0.0f };
    static const vector_t gravity = { 0.0f, 0.0f, 9.80665f };
    static float acc_temp_sum = 0.0f;

    acc_sum = vector_add(vector_add(acc_sum, _accel), gravity);
    acc_temp_sum += _imu_temperature;
    count++;

    if (count > 1000)
    {
        // The temperature bias is calculated using a least-squares regression.
        // This is computationally intensive, so it is done by the onboard computer in
        // fcu_io and shipped over to the flight controller.
        vector_t accel_temp_bias = {
          params->get_param_float(Params::PARAM_ACC_X_TEMP_COMP),
          params->get_param_float(Params::PARAM_ACC_Y_TEMP_COMP),
          params->get_param_float(Params::PARAM_ACC_Z_TEMP_COMP)
        };

        // Figure out the proper accel bias.
        // We have to consider the contribution of temperature during the calibration,
        // Which is why this line is so confusing. What we are doing, is first removing
        // the contribution of temperature to the measurements during the calibration,
        // Then we are dividing by the number of measurements.
        vector_t accel_bias = scalar_multiply(1.0f / (float)count, vector_sub(acc_sum, scalar_multiply(acc_temp_sum, accel_temp_bias)));

        // Sanity Check -
        // If the accelerometer is upside down or being spun around during the calibration,
        // then don't do anything
        if (sqrd_norm(accel_bias) < 4.5)
        {
            params->set_param_float(Params::PARAM_ACC_X_BIAS, accel_bias.x);
            params->set_param_float(Params::PARAM_ACC_Y_BIAS, accel_bias.y);
            params->set_param_float(Params::PARAM_ACC_Z_BIAS, accel_bias.z);
            Api::logMessage("IMU offsets captured", 0);

            // reset the estimated state
            estimator->reset_state();
            calibrating_acc_flag = false;
        } else
        {
            // check for bad _accel_scale
            if (sqrd_norm(accel_bias) > 4.5*4.5 && sqrd_norm(accel_bias) < 5.5*5.5)
            {
                Api::logMessage("Detected bad IMU accel scale value", 4);
                params->set_param_float(Params::PARAM_ACCEL_SCALE, 2.0f * params->get_param_float(Params::PARAM_ACCEL_SCALE));
                accel_scale *= params->get_param_float(Params::PARAM_ACCEL_SCALE);
                params->write_params();
            } else if (sqrd_norm(accel_bias) > 9.0f*9.0f && sqrd_norm(accel_bias) < 11.0*11.0)
            {
                Api::logMessage("Detected bad IMU accel scale value", 4);
                params->set_param_float(Params::PARAM_ACCEL_SCALE, 0.5f * params->get_param_float(Params::PARAM_ACCEL_SCALE));
                accel_scale *= params->get_param_float(Params::PARAM_ACCEL_SCALE);
                params->write_params();
            } else
            {
                Api::logMessage("Too much movement for IMU cal", 2);
                calibrating_acc_flag = false;
            }
        }

        // reset calibration in case we do it again
        count = 0;
        acc_sum.x = 0.0f;
        acc_sum.y = 0.0f;
        acc_sum.z = 0.0f;
        acc_temp_sum = 0.0f;
    }
}


void Sensors::correct_imu(void)
{
    // correct according to known biases and temperature compensation
    _accel.x -= params->get_param_float(Params::PARAM_ACC_X_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_X_BIAS);
    _accel.y -= params->get_param_float(Params::PARAM_ACC_Y_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_Y_BIAS);
    _accel.z -= params->get_param_float(Params::PARAM_ACC_Z_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_Z_BIAS);

    _gyro.x -= params->get_param_float(Params::PARAM_GYRO_X_BIAS);
    _gyro.y -= params->get_param_float(Params::PARAM_GYRO_Y_BIAS);
    _gyro.z -= params->get_param_float(Params::PARAM_GYRO_Z_BIAS);
}
