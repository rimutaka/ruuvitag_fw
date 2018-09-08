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
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

//Logging
#define NRF_LOG_MODULE_NAME "App"
#define NRF_LOG_LEVEL 3 // Using INFO level for debugging. I couldn't get DEBUG level to work.
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
static uint16_t acceleration_events = 0;    // Acceleration interrupts counter
static uint32_t ext_pin4_events = 0;        // Pin 4 interrupts counter
static uint16_t ad_cycles = 0;              // a counter for ad cycles
static uint16_t acceleration_events_b4 = 0; // counter from the previous cycle
static uint32_t ext_pin4_events_b4 = 0;     // counter from the previous cycle
static bool initAdSent = false;             // set to true once on start after sending an empty ad
//static uint32_t counter_cycles_since_on = 60; // decremented for a soft start
//static uint32_t counter_heartbeat_cycles = 0; // incremented with every main loop

//static ruuvi_sensor_t data;

static void main_timer_handler(void *p_context);
uint16_t getP30Voltage(void);

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

/** 
 * @brief Get all the readings from on-board sensors and package them into 
 */
static void packageStartupAdvertisement(void)
{
  static uint16_t vbat = 0;
  vbat = getBattery();    // Get the voltage supplied to the chip.
  encodeToRawFormat5OnStartup(data_buffer, vbat, BLE_TX_POWER); // All other fields will be zero.
  NRF_LOG_INFO("Startup ad packaged\r\n");
}

/** 
 * @brief Get all the readings from on-board sensors and package them into 
 */
static void packageSensorDataIntoAdvertisement(void)
{
  int32_t raw_t = 0;
  //uint32_t raw_p = 0; // removed to make space for event counter
  uint32_t raw_h = 0;
  lis2dh12_sensor_buffer_t buffer;

  // Get raw environmental data.
  bme280_read_measurements();
  raw_t = bme280_get_temperature();
  //raw_p = bme280_get_pressure();
  raw_h = bme280_get_humidity();

  // Get accelerometer data.
  lis2dh12_read_samples(&buffer, 1);

  // Get battery voltage
  static uint16_t vbat = 0;
  vbat = getBattery();

  // Prepare bytearray to broadcast.
  bme280_data_t environmental;
  environmental.temperature = raw_t;
  environmental.humidity = raw_h;
  environmental.pressure = ext_pin4_events; //raw_p; //This is a temp plug to pass it on. 5000 is bias that is taken out later.
  encodeToRawFormat5(data_buffer, &environmental, &buffer.sensor, acceleration_events, vbat, BLE_TX_POWER);

  NRF_LOG_INFO("Event ad packaged\r\n");
}

/**
 * @brief Blink the LEDs in response to events
 */
static void blinkLEDsOnEvents(void)
{
  // Turn green LED on from acceleration
  if (acceleration_events > 0)
  {
    NRF_LOG_INFO("Accel #%d\r\n", acceleration_events);
    nrf_gpio_pin_clear(LED_GREEN);
  }
  else
  {
    nrf_gpio_pin_set(LED_GREEN);
  }

  // Toggle red led for ext pin
  if (ext_pin4_events > 0)
  {
    NRF_LOG_INFO("Pin4 #%d\r\n", ext_pin4_events);
    nrf_gpio_pin_clear(LED_RED);
  }
  else
  {
    nrf_gpio_pin_set(LED_RED);
  }
}

/**@brief Timeout handler for the repeated timer
 */
void main_timer_handler(void *p_context)
{

  // get AIN6/P030 voltage and treat it as a pin4 event if the values are within ON range
  uint16_t p30voltage = getP30Voltage();
  NRF_LOG_INFO("%dv  ", p30voltage);
  /*
  if ((p30voltage > SAADC_PIN30_ON_MIN) && (p30voltage < SAADC_PIN30_ON_MAX))
    ext_pin4_events++;
  */

#if NRF_LOG_LEVEL > 2
  blinkLEDsOnEvents(); // only needed for debugging
#endif

  //are there new activations
  bool active = ((acceleration_events + ext_pin4_events) > 0);

  if (!initAdSent)
  {
    // send out a special ad indicating there was a boot event
    initAdSent = true;                   // mark as sent
    packageStartupAdvertisement();       // use a special ad to indicate a start event
    ad_cycles = AD_MAIN_LOOP_CYCLES + 1; // roll for the default number of cycles, but it can be overtaken by events
    bluetooth_advertising_start();
  }
  else if (active)
  {
    //Read sensor data only if there was an activation event
    packageSensorDataIntoAdvertisement();
  }

  //Re-start advertising only if there was a new acceleration event
  if (acceleration_events > acceleration_events_b4 || ext_pin4_events > ext_pin4_events_b4)
  {
    //Advertise for N cycles from now
    ad_cycles = AD_MAIN_LOOP_CYCLES + 1; // has to be + 1 because it clips the last cycle
    bluetooth_advertising_start();
    NRF_LOG_INFO("Ads started\r\n");
  }
  else if (ad_cycles == 1)
  {
    // Stop advertising
    bluetooth_advertising_stop();
    acceleration_events = 0; //Reset acceleration counter for the next cycle
    ext_pin4_events = 0;
    NRF_LOG_INFO("Ads stopped\r\n");
  }

  // Decrement ad counter all the way to zero
  if (ad_cycles > 0)
    ad_cycles--;

  // Store current value
  acceleration_events_b4 = acceleration_events;
  ext_pin4_events_b4 = ext_pin4_events;

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

  return NRF_SUCCESS;
}

/**
 * @brief Handle interrupt from an external pin.
 * @param message Ruuvi message, with source, destination, type and 8 byte payload. Ignore for now.
 **/
ret_code_t ext_int4_handler(const ruuvi_standard_message_t message)
{
  ext_pin4_events++;

  return NRF_SUCCESS;
}

// =============================== SAADC PIN30 =========================================== //

#if SAADC_PIN30_ENABLED

// Try to get sensor voltage in a safe manner
uint16_t getP30Voltage(void)
{
  // Can SAADC be used?
  if (nrf_drv_saadc_is_busy())
    return 0;

  // Take a blocking sample
  nrf_saadc_value_t voltage_level;
  ret_code_t err_code = nrf_drv_saadc_sample_convert(SAADC_PIN30_CHANNEL, &voltage_level);
  if (err_code != NRF_SUCCESS)
    return 0;

  // see drivers/battery.c for explanation of the values
  uint16_t voltage = voltage_level * 3.515625 + REVERSE_PROT_VOLT_DROP_MILLIVOLTS;

  //Return as voltage
  return voltage;
}

ret_code_t saadc_init(void)
{
  // enable it on pin 30
  nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

  // Call that init routine to avoid clashing with their initialisation
  battery_voltage_init();

  ret_code_t err_code = nrf_drv_saadc_channel_init(SAADC_PIN30_CHANNEL, &channel_config);

  return err_code;
}

#endif // End of SAADC PIN30

/**
 * @brief Function for application main entry.
 */
int main(void)
{
  ret_code_t err_code = 0; // counter, gets incremented by each failed init. It is 0 in the end if init was ok.
  init_sensors();

  // Initialize log.
  err_code |= init_log();

  // Setup leds. LEDs are active low, so setting high them turns leds off.
  err_code |= init_leds();     // INIT leds first and turn RED on.
  nrf_gpio_pin_clear(LED_RED); // If INIT fails at later stage, RED will stay lit.

  NRF_LOG_INFO("Inside main.\r\n");

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

// SAADC is optional
#if SAADC_PIN30_ENABLED
  err_code |= saadc_init();
#endif

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

  // Enable interrupts from an external sensor on P0.04
  err_code |= pin_interrupt_enable(INT_EXT4_PIN, NRF_GPIOTE_POLARITY_LOTOHI, ext_int4_handler);

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
