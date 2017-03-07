#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <turbotrig/turbovec.h>
#include "estimator.hpp"
#include "mode.hpp"
#include "param.hpp"
#include "sensors.hpp"
#include "controller.hpp"
#include "mixer.hpp"
#include "rc.hpp"
#include "board.hpp"
#include "api.hpp"
#include "commonstate.hpp"

int main(void)
{
    Board::init();

    // Read EEPROM to get initial params
    Params params;
    params.init_params();

    /***********************/
    /***  Hardware Setup ***/
    /***********************/

    // Initialize MAVlink Communication
    Api::init();

    /***********************/
    /***  Software Setup ***/
    /***********************/

    CommonState common_state;
    
    // Initialize Estimator
    // mat_exp <- greater accuracy, but adds ~90 us
    // quadratic_integration <- some additional accuracy, adds ~20 us
    // accelerometer correction <- if using angle mode, this is required, adds ~70 us
    Estimator estimator;
    estimator.init_estimator(&params, false, false, true);

    // Initialize Sensors
    Sensors sensors;
    sensors.init_sensors(&common_state, &estimator, &params);

    Mux mux;
    mux.init(&common_state, &params);

    // Initialize Motor Mixing
    Mixer mixer;
    mixer.init_mixing(&common_state, &params);
    mixer.init_PWM();

    Controller controller;
    controller.init(&common_state, &mux, &mixer, &estimator, &params);

    RC rc;
    rc.init(&common_state, &mux, &params);

    Mode mode;
    mode.init(&common_state, &sensors, &rc, &params);

    vector_t accel, gyro;
    uint64_t imu_time;

    // Main loop
    while (1)
    {
        /*********************/
        /***  Control Loop ***/
        /*********************/
        if (sensors.update_sensors()) // 595 | 591 | 590 us
        {
            // If I have new IMU data, then perform control
            sensors.getImuMeasurements(accel, gyro, imu_time);
            estimator.run_estimator(accel, gyro, imu_time); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
            controller.run_controller(); // 278 | 271
            mixer.mix_output(); // 16 | 13 us
        }

        /*********************/
        /***  Post-Process ***/
        /*********************/
        // internal timers figure out what and when to send
        Api::send(Board::micros()); // 165 | 27 | 2

        // receive mavlink messages
        Api::receive(); // 159 | 1 | 1

        // update the armed_states, an internal timer runs this at a fixed rate
        mode.check_mode(Board::micros()); // 108 | 1 | 1

        // get RC, an internal timer runs this every 20 ms (50 Hz)
        rc.receive_rc(Board::micros()); // 42 | 2 | 1

        // update commands (internal logic tells whether or not we should do anything or not)
        mux.mux_inputs(); // 6 | 1 | 1
    }
}
