/**
 * RP2040 FreeRTOS Template - App #2
 *
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.1.0
 * @licence   MIT
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_


// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
// CXX
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
// Pico SDK
#include "pico/stdlib.h"            // Includes `hardware_gpio.h`
#include "pico/binary_info.h"
#include "hardware/i2c.h"
// App
#include "i2c_utils.h"
#include "ht16k33.h"
#include "utils.h"
#include "mcp9808.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * CONSTANTS
 */
#define         RED_LED_PIN           20
#define         GRN_LED_PIN           15
#define         SENSOR_ALERT_PIN      16

/**
 * PROTOTYPES
 */
void setup();
void setup_led();
void setup_i2c();
void setup_gpio();

void gpio_cb(uint gpio, uint32_t events);

void led_on();
void led_off();
void led_set(bool state = true);

void led_task_pico(void* unused_arg);
void led_task_gpio(void* unused_arg);
void sensor_read_task(void* unused_arg);

void display_int(int number);
void display_tmp(double value);

void log_debug(const char* msg);
void log_device_info(void);




#ifdef __cplusplus
}           // extern "C"
#endif


#endif      // _MAIN_H_