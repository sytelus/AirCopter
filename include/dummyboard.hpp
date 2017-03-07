#pragma once

#include "board.hpp"

class DummyBoard : Board {
public:
    virtual void init() override {}
    virtual uint64_t micros() override { return 0; }
    virtual uint32_t millis() override {}
    virtual void initSensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback) override {}
    virtual bool isSensorPresent(SensorType type) override { return false; }
    virtual uint16_t pwmRead(int16_t channel) override { return 0; }
    virtual void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) override {}
    virtual void pwmWriteMotor(uint8_t index, uint16_t value) override {}
    virtual void setLed(uint8_t index, bool is_on) override {}
    virtual void toggleLed(uint8_t index) override {}
    virtual void initParams() override {}
    virtual bool readParams() override { return false;  }
    virtual bool writeParams(bool blink_led) override { return false; }
    virtual void initImu(uint16_t& acc1G, float& gyroScale, int boardVersion) override {}
    virtual void readAccel(int16_t accel_adc[3]) override {}
    virtual void readGyro(int16_t gyro_adc[3]) override {}
    virtual void readTemperature(int16_t& temp) override {}
    virtual void readBaro(float& altitude, float& pressure, float& temperature) override {}
    virtual void readDiffPressure(float& differential_pressure, float& temp, float& velocity) override {}
    virtual float readSonar() override { return 0; }
    virtual void readMag(int16_t mag_adc[3]) override {}
    virtual void delayMicros(uint32_t us) override {}
    virtual void delayMillis(uint32_t ms) override {}
};