#pragma once

#include <cstdint>
#include <functional>


namespace rosflight {

class Board {
public: //types
    enum SensorType {
        Imu = 0,
        Baro = 1,
        Mag = 2,
        Gps = 3,
        Sonar = 4,
        DiffPressure = 5
    };

public:
    virtual void init() = 0;
    virtual uint64_t micros() = 0;
    virtual uint32_t millis() = 0;
    virtual void initSensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback) = 0;
    virtual bool isSensorPresent(SensorType type) = 0;
    virtual uint16_t pwmRead(int16_t channel) = 0;
    virtual void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) = 0;
    virtual void pwmWriteMotor(uint8_t index, uint16_t value) = 0;
    virtual void setLed(uint8_t index, bool is_on) = 0;
    virtual void toggleLed(uint8_t index) = 0;
    virtual void initParams() = 0;
    virtual bool readParams() = 0;
    virtual bool writeParams(bool blink_led) = 0;
    virtual void initImu(uint16_t& acc1G, float& gyroScale, int boardVersion) = 0;
    virtual void readAccel(int16_t accel_adc[3]) = 0;
    virtual void readGyro(int16_t gyro_adc[3]) = 0;
    virtual void readTemperature(int16_t& temp) = 0;
    virtual void readBaro(float& altitude, float& pressure, float& temperature) = 0;
    virtual void readDiffPressure(float& differential_pressure, float& temp, float& velocity) = 0;
    virtual float readSonar() = 0;
    virtual void readMag(int16_t mag_adc[3]) = 0;
    virtual void delayMicros(uint32_t us) = 0;
    virtual void delayMillis(uint32_t ms) = 0;
};


} //namespace