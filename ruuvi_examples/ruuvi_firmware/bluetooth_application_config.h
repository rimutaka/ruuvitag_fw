#ifndef BLUETOOTH_APP_CONFIG
#define BLUETOOTH_APP_CONFIG

#define APPLICATION_DEVICE_NAME         "Sensr"                         /**< BLE name displayed in scan response. */
#define APPLICATION_DEVICE_NAME_LENGTH  5                               /**< number of characters in above string, excluding null */
#define APP_DEVICE_NAME                 APPLICATION_DEVICE_NAME         /**< TODO: Refactoring **/
#define APP_DEVICE_NAME_LENGTH          APPLICATION_DEVICE_NAME_LENGTH
#define APPLICATION_ADV_INTERVAL        1010                            /**< ms. Use value which is not exactly divisible by 1000 ms for Minew interoperability **/
#define APP_TX_POWER                    4                               /**< dBm **/
#define INIT_FWREV                      "2.2.1"                         /**< Github tag. Do not include specifiers such as "alpha" so you can accept ready binaries as they are **/
#define INIT_SWREV                      INIT_FWREV                      /**< FW and SW are same thing in this context **/

// milliseconds until main loop timer function is called. Other timers can bring
// application out of sleep at higher (or lower) interval.
#define MAIN_LOOP_INTERVAL_RAW      1000u  //How often main_timer_handler routine fires 
#define ADVERTISING_INTERVAL_RAW    200u   //How often BLE packets go out when advertising starts
#define AD_MAIN_LOOP_CYCLES         5      //Number of main_timer_handler cycles to advertise after a single activation  
#define HEARTBEAT_LOOP_CYCLES       3600u  // Number of main_timer_handler cycles between a single heartbeat ad
#define PIN30_ACTIVATION_TIMEOUT    0    // Number of cycles to wait after the pin goes high (LED is ON)
#define PIN31_ACTIVATION_TIMEOUT    120u   // Number of cycles to wait after the pin goes high (PIR Vdd is ON)

//Raw v2
#define RAW_DATA_LENGTH 24

/**
 *  BLE_GAP_ADV_TYPE_ADV_IND   0x00           Connectable, scannable
 *  BLE_GAP_ADV_TYPE_ADV_DIRECT_IND   0x01
 *  BLE_GAP_ADV_TYPE_ADV_SCAN_IND   0x02      Nonconnectable, scannable
 *  BLE_GAP_ADV_TYPE_ADV_NONCONN_IND   0x03   Nonconnectable, nonscannable
 */
#define APPLICATION_ADVERTISEMENT_TYPE 0x03
//Set to 0 if you don't want to include GATT connectivity. Remember to adjust advertisement type
#define APPLICATION_GATT 0

#endif
