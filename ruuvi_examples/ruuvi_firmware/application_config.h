#ifndef APPLICATION_CONFIG_H
#define APPLICATION_CONFIG_H

#include "bme280.h"
#include "lis2dh12.h"
// Milliseconds before new button press is accepted
#define DEBOUNCE_THRESHOLD 250u

// 1, 2, 4, 8, 16.
// Oversampling increases current consumption, but lowers noise.
// IIR lowers noise, but slows step response.
#define BME280_HUMIDITY_OVERSAMPLING    BME280_OVERSAMPLING_1
#define BME280_TEMPERATURE_OVERSAMPLING BME280_OVERSAMPLING_1
#define BME280_PRESSURE_OVERSAMPLING    BME280_OVERSAMPLING_1
#define BME280_IIR                      BME280_IIR_16
#define BME280_DELAY                    BME280_STANDBY_1000_MS

#define LIS2DH12_SCALE              LIS2DH12_SCALE2G
#define LIS2DH12_RESOLUTION         LIS2DH12_RES10BIT
#define LIS2DH12_SAMPLERATE_RAW     LIS2DH12_RATE_10
#define LIS2DH12_SAMPLERATE_URL     LIS2DH12_RATE_0

// LSB, i.e. scale and resolution affect the threshold //TODO: verify resolution
// 64 mg on 2G/10bit
#define LIS2DH12_ACTIVITY_THRESHOLD 0x04

// SAADC can be used to sample the state of some other sensor
#define SAADC_PIN30_ENABLED 1 // Compile voltage checks on pin 30 during every interval
#define SAADC_PIN30_CHANNEL 1 // Set the channel here to avoid conflicts with other parts of the code, e.g. battery
#define SAADC_PIN30_ON_MIN 200 // A min value range for an ON event
#define SAADC_PIN30_ON_MAX 1000 // A max value range for an ON event

#endif