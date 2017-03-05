#pragma once

#include <cstdint>
#include <functional>

class Board {
public:
    static void init()
    {
        //// Configure clock, this figures out HSE for hardware autodetect
        //SetSysClock(0);
        //systemInit();

        // Initialize I2c
        //i2cInit(I2CDEV_2);
    }

    static uint64_t Board::micros()
    {
        return 0;
    }

    static uint32_t millis()
    {
        return 0;
    }

    static void initSensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback)
    {
        //while(millis() < 50);
        //i2cWrite(0,0,0)

        //_baro_present = ms5611_init();
        //_mag_present = hmc5883lInit(get_param_int(PARAM_BOARD_REVISION));
        //_sonar_present = mb1242_init();
        //_diff_pressure_present = ms4525_init();
        // IMU
        //mpu6050_init(true, &acc1G, &gyro_scale, get_param_int(PARAM_BOARD_REVISION));
        //mpu6050_register_interrupt_cb(&imu_ISR);
    }

    enum SensorType {
        Imu = 0,
        Baro = 1,
        Mag = 2,
        Gps = 3,
        Sonar = 4,
        DiffPressure = 5
    };
    static bool isSensorPresent(SensorType type)
    {
        return false;

    }

    static uint16_t pwmRead(int16_t channel)
    {
        return 0;
    }

    static void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec)
    {

    }

    static void pwmWriteMotor(uint8_t index, uint16_t value)
    {

    }

    static void setLed(uint8_t index, bool is_on)
    {

    }
    static void toggleLed(uint8_t index)
    {

    }

    static void initParams()
    {

    }

    static bool readParams()
    {
        return true;
    }

    static bool writeParams(bool blink_led)
    {
        return true;

    }

    static void initImu(uint16_t& acc1G, float& gyroScale, int boardVersion)
    {

    }

    static void readAccel(int16_t accel_adc[3])
    {

    }

    static void readGyro(int16_t gyro_adc[3])
    {

    }

    static void readTemperature(int16_t& temp)
    {

    }

    static void readBaro(float& altitude, float& pressure, float& temperature)
    {

    }

    static void readDiffPressure(float& differential_pressure, float& temp, float& velocity)
    {

    }

    static float readSonar()
    {
        return 1E6;
    }

    static void readMag(int16_t mag_adc[3])
    {

    }

    static void delayMicroseconds(uint32_t us) {}
    static void delay(uint32_t ms) {}
};