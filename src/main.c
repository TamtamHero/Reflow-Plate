#include "main.h"
#include "ssd1306.h"
#include "font.h"
#include "image.h"
#include "math.h"

#include "hardware/adc.h"

#define DEBUG 1
/*
 * GLOBALS
 */
// This is the inter-task queue
volatile QueueHandle_t duty_cycle_queue = NULL;

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;

TaskHandle_t ui_task_handle = NULL, adc_task_handle = NULL, ntc_control_task_handle = NULL, ntc_pid_task_handle = NULL;

float temperature = 20.0;
float user_command = 0.0;
float res_target = 0;

float ntc_resistance = 0.0;

#define MAX_PTC_TEMP 260.0
#define MAX_NTC_RES 10000

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
#define PIN_PUSH_BUTTON 13
#define PIN_RELAY_PTC 14
#define PIN_ADC_POTENTIOMETER 26
#define PIN_ADC_THERMAL_SENSOR 27

#define ADC_OFFSET 26
#define PIN_TO_ADC(pin) ((pin)-ADC_OFFSET)
#define JUST_HOTTER(n1, n2, percent) (n1 >= (n2 - (n2*percent/100)))
#define JUST_COLDER(n1, n2, percent) (n1 <= (n2 + (n2*percent/100)))
#define EQ_APPROX(n1, n2, percent) (JUST_HOTTER(n1, n2, percent) && JUST_COLDER(n1, n2, percent))
#define MUCH_HOTTER(n1, n2, percent) (n1 < (n2 - (n2*percent/100)))
#define MUCH_COLDER(n1, n2, percent) (n1 > (n2 + (n2*percent/100)))
#define HOTTER(n1, n2, percent) (n1 < (n2 - (n2*percent/100)))
#define COLDER(n1, n2, percent) (n1 > (n2 + (n2*percent/100)))

float ntc_res_to_kelvin(float r){
    const float A = 0.0007250405468, B = 0.0002835414096, C = 0.00000001168632417;
    return 1 / (A + B*log(r) + C*pow(log(r), 3));
}

float ntc_res_to_celsius(float r){
    return ntc_res_to_kelvin(r) - 273.15;
}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void ui_task(void* unused_arg) {

    // setup GPIOs
    i2c_init(i2c1, 400000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // setup ui
    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(&disp);

    char buf[8];
    float prev_temp = temperature;

    TickType_t delay = 100 / portTICK_PERIOD_MS;
    while(true){
        if(prev_temp != temperature){
            snprintf(buf, sizeof(buf), "%.0f%%", user_command);
            ssd1306_draw_string_with_font(&disp, 2, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f", res_target);
            ssd1306_draw_string_with_font(&disp, 34, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f C°", ntc_res_to_celsius(res_target));
            ssd1306_draw_string_with_font(&disp, 84, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f", temperature);
            ssd1306_draw_string_with_font(&disp, 8, 24, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f C°", ntc_res_to_celsius(temperature));
            ssd1306_draw_string_with_font(&disp, 8, 42, 1, font_8x5, buf);
            ssd1306_show(&disp);
            ssd1306_clear(&disp);
            prev_temp = temperature;
        }
        vTaskDelay(delay);
    }

}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void adc_task(void* unused_arg) {

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(PIN_ADC_THERMAL_SENSOR);
    adc_gpio_init(PIN_ADC_POTENTIOMETER);
    const float conversion_factor = 3.3f / (1 << 12);
    const float v_ref = 3.3;
    const float r1 = 3230;
    TickType_t delay = 50 / portTICK_PERIOD_MS;
    while(true){
        float mean_temp = 0;
        float mean_user_command = 0;
        for (size_t i = 0; i < 20; i++)
        {
            // Select ADC input 1 (PIN_ADC_POTENTIOMETER, potentiometer sensing)
            adc_select_input(PIN_TO_ADC(PIN_ADC_POTENTIOMETER));
            float voltage = adc_read() * conversion_factor;
            mean_user_command += ((voltage * 100) / v_ref)/20.0;
            // printf("Voltage: %f V, command: %f%%, %f %f \n", voltage, mean_user_command, (voltage * 100), ((voltage * 100) / v_ref));

            // Select ADC input 2 (PIN_ADC_THERMAL_SENSOR, temperature sensing)
            adc_select_input(PIN_TO_ADC(PIN_ADC_THERMAL_SENSOR));
            voltage = adc_read() * conversion_factor;
            float thermistance = (r1 * voltage)/(v_ref - voltage);
            mean_temp += thermistance/20.0;
            // printf("Voltage: %f V\n", voltage);
            vTaskDelay(delay);
        }
        user_command = mean_user_command;
        temperature = mean_temp;
        ntc_resistance = temperature;
    }
}

// /**
//  * @brief Repeatedly flash an LED connected to GPIO pin 20
//  *        based on the value passed via the inter-task queue.
//  */
// void ptc_control_task(void* unused_arg) {

//     // temperature setpoint for PTC
//     float temp_setpoint = 0;
//     uint8_t duty_cycle = 0;

//     const float P = 1, I = 0.1, D = 0.0001;
//     float prev_temp = 0, prev_error = 0;
//     float temp_error = 0, error_slope = 0, error_integral = 0;
//     while(true){
//         temp_setpoint = user_command * MAX_PTC_TEMP / 100.0;

//         temp_error = temp_setpoint - temperature;
//         error_slope = (temp_error - prev_error) / time_interval;
//         error_integral += (prev_error + temp_error) * time_interval / 2;
//         float unbound_duty_cycle = P * temp_error + I * error_integral + D * error_slope;
//         duty_cycle = (uint8_t)(unbound_duty_cycle > 100 ? 100 : unbound_duty_cycle);
//     }
// }

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void ntc_pid_task(void* unused_arg) {

    // resistance setpoint for NTC
    float res_setpoint = 0;

    float duty_cycle = 0.0;
    const float P = 0.02, I = 0.0017, D = 0.2;
    float prev_res = 0, prev_error = 0;
    float res_error = 0, error_slope = 0, error_integral = 0, error_percent = 0;
    // help guess if we're in heating or cooldown mode on first iteration
    float prev_res_setpoint = ntc_resistance;
    bool cooldown = false, heatup = true;
    float time_interval = 2000;
    TickType_t delay = time_interval / portTICK_PERIOD_MS;
    uint i = 0;
    // wait until system is ready
    while(user_command == 0){
        vTaskDelay(delay);
    }
    // init prev_error so the first error slope computed doesn't jump on first iteration
    res_setpoint = user_command * MAX_NTC_RES / 100.0;
    res_error = ntc_resistance - res_setpoint;
    prev_error = res_error;
    while(true){
        res_setpoint = user_command * MAX_NTC_RES / 100.0;
        res_target = res_setpoint;

        res_error = ntc_resistance - res_setpoint;
        error_percent = abs(res_error * 100 / res_setpoint);
        error_slope = (res_error - prev_error) / (time_interval / 1000);

        // if the new comand is a higher res (lower temp) than the previous +-1%, let's drop the integral part and reset error slope
        if(COLDER(res_setpoint, prev_res_setpoint, 10) && MUCH_HOTTER(ntc_resistance, res_setpoint, 2)){
            cooldown = true;
            heatup = false;
            error_integral = 0.0;
            error_slope= 0.0;
            printf("cd! res_setpoint: %f, prev_setpoint: %f, limit: %f, current: %f\n", res_setpoint, prev_res_setpoint, prev_res_setpoint + (prev_res_setpoint/200.0), ntc_resistance);
        }
        else if (HOTTER(res_setpoint, prev_res_setpoint, 10) && MUCH_COLDER(ntc_resistance, res_setpoint, 2)){
            heatup = true;
            cooldown = false;
            error_slope = 0.0;
            printf("heatup! res_setpoint: %f, prev_setpoint: %f, limit: %f, current: %f\n", res_setpoint, prev_res_setpoint, prev_res_setpoint - (prev_res_setpoint/200.0), ntc_resistance);
        }
        if(cooldown == true && JUST_HOTTER(ntc_resistance, res_setpoint, 10)){
            cooldown = false;
            printf("no cd!\n");
        }
        if(heatup == true && JUST_COLDER(ntc_resistance, res_setpoint, 10)){
            heatup = false;
            printf("no heatup!\n");
        }
        if(cooldown == false && heatup == false){
            error_integral += (prev_error + res_error) * (time_interval / 1000) / 2;
            if(error_integral < 0)
                error_integral = 0;
        }
        else if(heatup == true){
            // we still want a bit of I increase when heating up (/5 vs normal I coeff)
            error_integral += (prev_error + res_error) / 1 * (time_interval / 1000) / 2;
        }

        duty_cycle = P * res_error + I * error_integral + D * error_slope;
        duty_cycle = duty_cycle > 100 ? 100 : duty_cycle < 0 ? 0 : duty_cycle;

        xQueueSend(duty_cycle_queue, &duty_cycle, portMAX_DELAY);

        // update state for next loop iteration
        prev_error = res_error;
        prev_res_setpoint = res_setpoint;
        i++;

        char * status = cooldown ? "-C" : heatup ? "+H" : "..";
        printf("%u | duty: %.1f%%, P: %.1f, I: %.1f(%s), D: %.1f - res_err: %.1f\n", i, duty_cycle, P*res_error, I*error_integral, status, D*error_slope, res_error);
        // duty = %
        // duty_cycle = user_command;
        vTaskDelay(delay);
    }
}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void ntc_control_task(void* unused_arg) {

    gpio_init(PIN_RELAY_PTC);
    gpio_set_dir(PIN_RELAY_PTC, GPIO_OUT);
    const float tick_ms = 20; // cyle is 20 * 100 = 2s long
    float duty_cycle;
    while(true){
        if (xQueueReceive(duty_cycle_queue, &duty_cycle, portMAX_DELAY) == pdPASS) {
            TickType_t on_delay = duty_cycle*tick_ms / portTICK_PERIOD_MS;
            TickType_t off_delay = (100-duty_cycle)*tick_ms / portTICK_PERIOD_MS;
            gpio_put(PIN_RELAY_PTC, true);
            vTaskDelay(on_delay);
            gpio_put(PIN_RELAY_PTC, false);
            vTaskDelay(off_delay);
        }
    }
}


/**
 * @brief Generate and print a debug message from a supplied string.
 *
 * @param msg: The base message to which `[DEBUG]` will be prefixed.
 */
void log_debug(const char* msg) {

#ifdef DEBUG
    uint msg_length = 9 + strlen(msg);
    char* sprintf_buffer = malloc(msg_length);
    sprintf(sprintf_buffer, "[DEBUG] %s\n", msg);
    printf("%s", sprintf_buffer);
    free(sprintf_buffer);
#endif
}


/**
 * @brief Show basic device info.
 */
void log_device_info(void) {

    printf("App: %s %s (%i)\n", APP_NAME, APP_VERSION, BUILD_NUM);
}


/*
 * RUNTIME START
 */
int main() {

    // Enable STDIO
#ifdef DEBUG
    stdio_usb_init();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);

    // Log app info
    log_device_info();
#endif

    BaseType_t tasks_status[] = {
        xTaskCreate(ui_task,
            "UI_TASK",
            2048,
            NULL,
            1,
            &ui_task_handle),
        xTaskCreate(adc_task,
            "adc_TASK",
            2048,
            NULL,
            1,
            &adc_task_handle),
        xTaskCreate(ntc_pid_task,
            "ntc_pid_TASK",
            2048,
            NULL,
            1,
            &ntc_pid_task_handle),
        xTaskCreate(ntc_control_task,
            "ntc_control_TASK",
            2048,
            NULL,
            1,
            &ntc_control_task_handle)
    };

    // Set up the event queue
    duty_cycle_queue = xQueueCreate(4, sizeof(float));

    for (size_t i = 0; i < sizeof(tasks_status)/sizeof(tasks_status[0]); i++)
    {
        if (tasks_status[i] != pdPASS) {
            printf("Task number %u error: %u\n", i, tasks_status[i]);
        }
    }

    vTaskStartScheduler();

    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
