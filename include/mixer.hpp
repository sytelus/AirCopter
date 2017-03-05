#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "CommonState.hpp"
#include "param.hpp"

class Mixer {
public:
    typedef struct
    {
        float F;
        float x;
        float y;
        float z;
    } command_t;

    typedef enum
    {
        QUADCOPTER_PLUS = 0,
        QUADCOPTER_X = 1,
        QUADCOPTER_H,
        TRICOPTER,
        Y6,
        FIXEDWING,
        NUM_MIXERS
    } mixer_type_t;

public:
    void init_mixing(CommonState* _common_state, Params* _params);
    void init_PWM();
    void mix_output();
    command_t& getCommand() { return _command; }

private:
    void write_motor(uint8_t index, int32_t value);
    void Mixer::write_servo(uint8_t index, int32_t value);

private:
    typedef enum
    {
        NONE, // None
        S, // Servo
        M, // Motor
        G // GPIO
    } output_type_t;

    typedef struct
    {
        output_type_t output_type[8];
        float F[8];
        float x[8];
        float y[8];
        float z[8];
    } mixer_t;

    int32_t _GPIO_outputs[8];
    int32_t prescaled_outputs[8];
    int32_t _outputs[8];
    command_t _command;
    output_type_t _GPIO_output_type[8];

    CommonState* common_state;
    Params* params;

    constexpr static mixer_t quadcopter_plus_mixing =
    {
        {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

        { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
        { 0.0f, -1.0f,  1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
        {-1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
        {-1.0f,  1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
    };


    constexpr static mixer_t quadcopter_x_mixing =
    {
        {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

        { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
        {-1.0f,-1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
        {-1.0f, 1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
        {-1.0f, 1.0f, 1.0f,-1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
    };

    constexpr static mixer_t quadcopter_h_mixing =
    {
        {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

        { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
        {-1057, -943, 1057,  943,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
        {-1005,  995,-1005,  995,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
        {-1.0f, 1.0f, 1.0f,-1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
    };

    constexpr static mixer_t fixedwing_mixing =
    {
        {S, S, M, S, NONE, NONE, NONE, NONE},

        { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
        { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
        { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
        { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
    };

    constexpr static mixer_t tricopter_mixing =
    {
        {M, M, M, S, NONE, NONE, NONE, NONE},

        { 1.0f,     1.0f,   1.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
        {-1.0f,     1.0f,   0.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
        {-0.667f,  -0.667f, 1.333f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
        { 0.0f,     0.0f,   0.0f,   1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
    };

    constexpr static mixer_t Y6_mixing =
    {
        {M, M, M, M, M, M, NONE, NONE},
        { 1.0f,   1.0f,    1.0f,    1.0f,    1.0f,    1.0f,   0.0f, 0.0f}, // F Mix
        { 0.0f,  -1.0f,    1.0f,    0.0f,   -1.0f,    1.0f,   0.0f, 0.0f}, // X Mix
        {-1.333f, 0.667f,  0.667f, -1.333f,  0.667f,  0.667f, 0.0f, 0.0f}, // Y Mix
        {-1.0f,   1.0f,    1.0f,    1.0f,   -1.0f,   -1.0f,   0.0f, 0.0f}  // Z Mix
    };

    constexpr static mixer_t const *array_of_mixers[NUM_MIXERS] =
    {
        &quadcopter_plus_mixing,
        &quadcopter_x_mixing,
        &quadcopter_h_mixing,
        &tricopter_mixing,
        &Y6_mixing,
        &fixedwing_mixing
    };

    mixer_t mixer_to_use;
};