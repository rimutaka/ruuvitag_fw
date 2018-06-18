/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * Firmware for the RuuviTag B with weather-station functionality.
 */

// STDLIB
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Nordic SDK
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// BSP
//#define BSP_SIMPLE
#include "bsp.h"

// Drivers
#include "lis2dh12.h"
#include "lis2dh12_acceleration_handler.h"
#include "bme280.h"
#include "battery.h"
#include "bluetooth_core.h"
#include "eddystone.h"
#include "pin_interrupt.h"
#include "rtc.h"
#include "application_config.h"

// Libraries
#include "base64.h"
#include "sensortag.h"

// Init
#include "init.h"

// Configuration
#include "bluetooth_config.h"

// Constants
#define DEAD_BEEF 0xDEADBEEF //!< Value used as error code on stack dump, can be used to identify stack location on stack unwind.

// ID for main loop timer.
APP_TIMER_DEF(main_timer_id); // Creates timer id for our program.

// Payload requires 9 characters
//static char url_buffer[URL_BASE_LENGTH + URL_DATA_LENGTH] = URL_BASE;
static uint8_t data_buffer[RAW_DATA_LENGTH] = {0};
//static bool model_plus = false;     // Flag for sensors available
//static bool highres = true;        // Flag for used mode
static uint16_t acceleration_events = 0;
static uint16_t ad_cycles = 0;              //a counter for ad cycles
static uint16_t acceleration_events_b4 = 0; //counter from the previous cycle

static ruuvi_sensor_t data;

static void main_timer_handler(void *p_context);

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

static void updateAdvertisement(void)
{
  ret_code_t err_code = NRF_SUCCESS;
  err_code |= bluetooth_set_manufacturer_data(data_buffer, sizeof(data_buffer));
}

/**@brief Timeout handler for the repeated timer
 */
void main_timer_handler(void *p_context)
{

#if defined(LED_INDICATE_BROADCAST) && LED_INDICATE_BROADCAST > 0

  if (acceleration_events > 0)
  {
    nrf_gpio_pin_clear(LED_GREEN);
  }
  else
  {
    nrf_gpio_pin_set(LED_GREEN);
  }

#endif

  //Read sensor data only if there was an acceleration event
  if (acceleration_events > 0)
  {
    int32_t raw_t = 0;
    uint32_t raw_p = 0;
    uint32_t raw_h = 0;
    lis2dh12_sensor_buffer_t buffer;
    int32_t acc[3] = {0};

    // Get raw environmental data.
    bme280_read_measurements();
    raw_t = bme280_get_temperature();
    raw_p = bme280_get_pressure();
    raw_h = bme280_get_humidity();

    // Get accelerometer data.
    lis2dh12_read_samples(&buffer, 1);
    acc[0] = buffer.sensor.x;
    acc[1] = buffer.sensor.y;
    acc[2] = buffer.sensor.z;

    // Get battery voltage
    static uint16_t vbat = 0;
    vbat = getBattery();

    // Embed data into structure for parsing.
    parseSensorData(&data, raw_t, raw_p, raw_h, vbat, acc);
    NRF_LOG_DEBUG("temperature: %d, pressure: %d, humidity: %d x: %d y: %d z: %d\r\n", raw_t, raw_p, raw_h, acc[0], acc[1], acc[2]);
    NRF_LOG_DEBUG("VBAT: %d send %d \r\n", vbat, data.vbat);

    // Prepare bytearray to broadcast.
    bme280_data_t environmental;
    environmental.temperature = raw_t;
    environmental.humidity = raw_h;
    environmental.pressure = raw_p;
    encodeToRawFormat5(data_buffer, &environmental, &buffer.sensor, acceleration_events, vbat, BLE_TX_POWER);
  }

  NRF_LOG_DEBUG("Ad c: %d, ace: %d, aceb4: %d\r\n", ad_cycles, acceleration_events, acceleration_events_b4);

  //Advertise only if there was a new acceleration event
  if (acceleration_events > acceleration_events_b4)
  {
    ad_cycles = AD_MAIN_LOOP_CYCLES; //Advertise for N cycles from now
    NRF_LOG_DEBUG("Start ads: %d\r\n", ad_cycles);
    bluetooth_advertising_start();
  }
  else if (ad_cycles == 0 && acceleration_events > 0)
  {
    NRF_LOG_DEBUG("Stop ads: %d\r\n", ad_cycles);
    bluetooth_advertising_stop();
    acceleration_events = 0; //Reset acceleration counter for the next cycle
  }

  if (ad_cycles > 0)
    ad_cycles--;
  acceleration_events_b4 = acceleration_events; //Store it for comparing in the next cycle

  updateAdvertisement();
  watchdog_feed();
}

/**
 * @brief Handle interrupt from lis2dh12.
 * Never do long actions, such as sensor reads in interrupt context.
 * Using peripherals in interrupt is also risky,
 * as peripherals might require interrupts for their function.
 *
 *  @param message Ruuvi message, with source, destination, type and 8 byte payload. Ignore for now.
 **/
ret_code_t lis2dh12_int2_handler(const ruuvi_standard_message_t message)
{
  NRF_LOG_DEBUG("Accelerometer interrupt to pin 2\r\n");
  acceleration_events++;
  /*
    app_sched_event_put ((void*)(&message),
                         sizeof(message),
                         lis2dh12_scheduler_event_handler);
    */
  return NRF_SUCCESS;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
  NRF_LOG_INFO("Initialising.\r\n");
  NRF_LOG_DEBUG("Debug logging ON.\r\n");
  ret_code_t err_code = 0; // counter, gets incremented by each failed init. It is 0 in the end if init was ok.
  init_sensors();

  // Initialize log.
  err_code |= init_log();

  // Setup leds. LEDs are active low, so setting high them turns leds off.
  err_code |= init_leds();     // INIT leds first and turn RED on.
  nrf_gpio_pin_clear(LED_RED); // If INIT fails at later stage, RED will stay lit.

  //Init NFC ASAP in case we're waking from deep sleep via NFC (todo)
  //err_code |= init_nfc();

  // Initialize BLE Stack. Required in all applications for timer operation.
  err_code |= init_ble();
  bluetooth_configure_advertisement_type(APPLICATION_ADVERTISEMENT_TYPE);
  bluetooth_tx_power_set(BLE_TX_POWER);
  bluetooth_configure_advertising_interval(ADVERTISING_INTERVAL_RAW);

  // Initialize the application timer module.
  err_code |= init_timer(main_timer_id, MAIN_LOOP_INTERVAL_RAW, main_timer_handler);

  // Initialize RTC.
  err_code |= init_rtc();

  // Start interrupts.
  err_code |= pin_interrupt_init();

  // Interrupt handler is defined in lis2dh12_acceleration_handler.c, reads the buffer and passes the data onwards to application as configured.
  // Try using PROPRIETARY as a target of accelerometer to implement your own logic.
  err_code |= pin_interrupt_enable(INT_ACC1_PIN, NRF_GPIOTE_POLARITY_LOTOHI, lis2dh12_int1_handler);

  // Initialize BME 280 and lis2dh12.

  // Clear memory.
  lis2dh12_reset();
  // Wait for reboot.
  nrf_delay_ms(10);
  // Enable XYZ axes.
  lis2dh12_enable();
  lis2dh12_set_scale(LIS2DH12_SCALE);
  // Sample rate 10 for activity detection.
  lis2dh12_set_sample_rate(LIS2DH12_SAMPLERATE_RAW);
  lis2dh12_set_resolution(LIS2DH12_RESOLUTION);

//XXX If you read this, I'm sorry about line below.
#include "lis2dh12_registers.h"
  // Configure activity interrupt - TODO: Implement in driver, add tests.
  uint8_t ctrl[1];
  // Enable high-pass for Interrupt function 2.
  //CTRLREG2 = 0x02
  ctrl[0] = LIS2DH12_HPIS2_MASK;
  lis2dh12_write_register(LIS2DH12_CTRL_REG2, ctrl, 1);

  // Enable interrupt 2 on X-Y-Z HI/LO.
  //INT2_CFG = 0x7F
  ctrl[0] = 0x7F;
  lis2dh12_write_register(LIS2DH12_INT2_CFG, ctrl, 1);
  // Interrupt on 64 mg+ (highpassed, +/-).
  //INT2_THS= 0x04 // 4 LSB = 64 mg @2G scale
  ctrl[0] = LIS2DH12_ACTIVITY_THRESHOLD;
  lis2dh12_write_register(LIS2DH12_INT2_THS, ctrl, 1);

  // Enable LOTOHI interrupt on nRF52.
  err_code |= pin_interrupt_enable(INT_ACC2_PIN, NRF_GPIOTE_POLARITY_LOTOHI, lis2dh12_int2_handler);

  // Enable Interrupt function 2 on LIS interrupt pin 2 (stays high for 1/ODR).
  lis2dh12_set_interrupts(LIS2DH12_I2C_INT2_MASK, 2);

  // Setup BME280 - oversampling must be set for each used sensor.
  bme280_set_oversampling_hum(BME280_HUMIDITY_OVERSAMPLING);
  bme280_set_oversampling_temp(BME280_TEMPERATURE_OVERSAMPLING);
  bme280_set_oversampling_press(BME280_PRESSURE_OVERSAMPLING);
  bme280_set_iir(BME280_IIR);
  bme280_set_interval(BME280_DELAY);
  bme280_set_mode(BME280_MODE_NORMAL);
  NRF_LOG_DEBUG("BME280 configuration done\r\n");

  // Visually display init status. Hangs if there was an error, waits 3 seconds on success.
  init_blink_status(err_code);

  nrf_gpio_pin_set(LED_RED); // Turn RED led off.

  // Delay before advertising so we get valid data on first packet
  nrf_delay_ms(MAIN_LOOP_INTERVAL_RAW + 100);

  // Init ok, start watchdog with default wdt event handler (reset).
  init_watchdog(NULL);

  // Enter main loop.
  for (;;)
  {
    app_sched_execute();
    power_manage();
  }
}
