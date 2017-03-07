#include "firmware.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <turbotrig/turbovec.h>

Firmware::Firmware(Board* _board)
    : board(_board)
{
}

void Firmware::setup()
{
    board->init();

    //initialize parameters source such as EPROM
    params.init(board);

    // Initialize communication stack such as MavLink
    Api::init();


    // Initialize Estimator
    // mat_exp <- greater accuracy, but adds ~90 us
    // quadratic_integration <- some additional accuracy, adds ~20 us
    // accelerometer correction <- if using angle mode, this is required, adds ~70 us
    estimator.init_estimator(&params, false, false, true);

    // Initialize Sensors
    sensors.init_sensors(&common_state, board, &estimator, &params);

    mux.init(&common_state, board, &params);

    // Initialize Motor Mixing
    mixer.init(&common_state, board, &params);
    mixer.init_PWM();

    controller.init(&common_state, board, &mux, &mixer, &estimator, &params);

    rc.init(&common_state, board, &mux, &params);

    mode.init(&common_state, &sensors, &rc, &params);

}

void Firmware::loop()
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
    Api::send(board->micros()); // 165 | 27 | 2

    // receive mavlink messages
    Api::receive(); // 159 | 1 | 1

    // update the armed_states, an internal timer runs this at a fixed rate
    mode.check_mode(board->micros()); // 108 | 1 | 1

    // get RC, an internal timer runs this every 20 ms (50 Hz)
    rc.receive_rc(board->micros()); // 42 | 2 | 1

    // update commands (internal logic tells whether or not we should do anything or not)
    mux.mux_inputs(); // 6 | 1 | 1
}
