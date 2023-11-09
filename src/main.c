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

float ptc_cur_temp_k = 293.15;
float user_command = 0.0;
float temp_target = 0;
float res_target = 0;

float ntc_resistance = 0.0;

#define PIN_TO_ADC(pin) ((pin)-ADC_OFFSET)
// #define JUST_HOTTER(n1, n2, percent) (n1 >= (n2 - (n2*percent/100)))
// #define JUST_COLDER(n1, n2, percent) (n1 <= (n2 + (n2*percent/100)))
// #define EQ_APPROX(n1, n2, percent) (JUST_HOTTER(n1, n2, percent) && JUST_COLDER(n1, n2, percent))
// #define HOTTER(n1, n2, percent) (n1 < (n2 - (n2*percent/100)))
// #define COLDER(n1, n2, percent) (n1 > (n2 + (n2*percent/100)))
#define JUST_COLDER(n1, n2, percent) (n1 >= (n2 - (n2*percent/100)))
#define JUST_HOTTER(n1, n2, percent) (n1 <= (n2 + (n2*percent/100)))
#define EQ_APPROX(n1, n2, threshold) (n1 >= n2 - threshold && n1 < n2 + threshold)
#define COLDER(n1, n2, percent) (n1 < (n2 - (n2*percent/100)))
#define HOTTER(n1, n2, percent) (n1 > (n2 + (n2*percent/100)))
#define FIT_TO_INTERVAL(x, a, b) (x < a ? a : x > b ? b : x)
#define KELVIN_TO_CELSIUS(t) (t - 273.15)
#define CELSIUS_TO_KELVIN(t) (t + 273.15)

#define MAX_PTC_TEMP CELSIUS_TO_KELVIN(260)
#define MAX_NTC_RES 15400

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
#define PIN_PUSH_BUTTON 13
#define PIN_RELAY_PTC 14
#define PIN_ADC_POTENTIOMETER 26
#define PIN_ADC_THERMAL_SENSOR 27

#define ADC_OFFSET 26

float ntc_res_to_kelvin(float r){
    const float A = 0.0007250405468, B = 0.0002835414096, C = 0.00000001168632417;
    return 1 / (A + B*log(r) + C*pow(log(r), 3));
}

float ntc_res_to_celsius(float r){
    return KELVIN_TO_CELSIUS(ntc_res_to_kelvin(r));
}

float kelvin_to_ntc_res(float t){
    const float A = 0.0007250405468, B = 0.0002835414096, C = 0.00000001168632417;
    float x = (A-1/t)/C;
    float y = sqrt(pow(B/(3*C), 3) + pow(x,2)/4);
    return exp(cbrt(y-x/2)-cbrt(y+x/2));
}

float roundToDecimal(float value, int decimalPlaces) {
    float multiplier = powf(10.0f, decimalPlaces);
    return roundf(value * multiplier) / multiplier;
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

    TickType_t delay = 100 / portTICK_PERIOD_MS;
    while(true){
            snprintf(buf, sizeof(buf), "%.0f%%", user_command);
            ssd1306_draw_string_with_font(&disp, 2, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.1f", res_target);
            ssd1306_draw_string_with_font(&disp, 34, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.1f C°", temp_target);
            ssd1306_draw_string_with_font(&disp, 84, 2, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f", ntc_resistance);
            ssd1306_draw_string_with_font(&disp, 8, 24, 1, font_8x5, buf);
            snprintf(buf, sizeof(buf), "%.2f C°", ntc_res_to_celsius(ntc_resistance));
            ssd1306_draw_string_with_font(&disp, 8, 42, 1, font_8x5, buf);
            ssd1306_show(&disp);
            ssd1306_clear(&disp);
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
    const uint8_t sampling_count_ntc = 20;
    const uint8_t sampling_count_potentiometer = 5;
    const float sampling_duration = 1000;
    float prev_user_command = -1;
    TickType_t delay = (sampling_duration/sampling_count_ntc) / portTICK_PERIOD_MS;
    while(true){
        float mean_ntc_res = 0;
        float mean_user_command = 0;
        for (size_t i = 0; i < sampling_count_ntc; i++)
        {
            // Select ADC input 1 (PIN_ADC_POTENTIOMETER, potentiometer sensing)
            adc_select_input(PIN_TO_ADC(PIN_ADC_POTENTIOMETER));
            float voltage = adc_read() * conversion_factor;
            // potentiometer sometime ahs issues with voltage close to 0 or 3v3, so we cut 0.05v on the edge
            voltage -= 0.1;
            mean_user_command += FIT_TO_INTERVAL(((voltage * 100) / (v_ref-0.1)), 0, 100)/sampling_count_potentiometer;
            if(i%sampling_count_potentiometer == sampling_count_potentiometer-1){
                user_command = roundToDecimal(mean_user_command, 1);
                // update target temperature and ntc resistance only if there is a significant change to user command
                if(!EQ_APPROX(prev_user_command, user_command, 0.2)){
                    float setpoint = user_command * KELVIN_TO_CELSIUS(MAX_PTC_TEMP) / 100.0;
                    temp_target =setpoint;
                    res_target = kelvin_to_ntc_res(CELSIUS_TO_KELVIN(setpoint));
                    prev_user_command = user_command;
                }
                // reset tmp variable for next sampling iterations
                mean_user_command = 0;
            }
            // printf("Voltage: %f V, command: %f%%, %f %f \n", voltage, mean_user_command, (voltage * 100), ((voltage * 100) / v_ref));

            // Select ADC input 2 (PIN_ADC_THERMAL_SENSOR, temperature sensing)
            adc_select_input(PIN_TO_ADC(PIN_ADC_THERMAL_SENSOR));
            voltage = adc_read() * conversion_factor;
            float thermistance = (r1 * voltage)/(v_ref - voltage);
            mean_ntc_res += thermistance/sampling_count_ntc;
            // printf("Voltage: %f V\n", voltage);
            vTaskDelay(delay);
        }
        ntc_resistance = mean_ntc_res;
        ptc_cur_temp_k = ntc_res_to_kelvin(ntc_resistance);
    }
}

// /**
//  * @brief Repeatedly flash an LED connected to GPIO pin 20
//  *        based on the value passed via the inter-task queue.
//  */
void ptc_pid_task(void* unused_arg) {

    // temperature setpoint in kelvin for PTC
    float setpoint = 0;

    float duty_cycle = 0.0;
    float P = 1, I = 0.0035, D = 4;
    float prev_temp = 0, prev_error = 0;
    float temp_error = 0, error_slope = 0, error_integral = 0, error_percent = 0;
    // help guess if we're in heating or cooldown mode on first iteration
    float prev_setpoint = ptc_cur_temp_k;
    bool cooldown = false, heatup = true;
    float time_interval = 2000;
    TickType_t delay = time_interval / portTICK_PERIOD_MS;
    uint i = 0;
    // wait until system is ready
    while(user_command == 0){
        vTaskDelay(delay);
    }
    // init prev_error so the first error slope computed doesn't jump on first iteration
    setpoint = CELSIUS_TO_KELVIN(user_command * KELVIN_TO_CELSIUS(MAX_PTC_TEMP) / 100.0);
    temp_error = setpoint - ptc_cur_temp_k;
    prev_error = temp_error;
    while(true){
        setpoint = CELSIUS_TO_KELVIN(user_command * KELVIN_TO_CELSIUS(MAX_PTC_TEMP) / 100.0);

        temp_error = setpoint - ptc_cur_temp_k;
        error_percent = abs(temp_error * 100 / setpoint);
        error_slope = (temp_error - prev_error) / (time_interval / 1000);

        // if the new comand is a higher res (lower temp) than the previous +-1%, let's drop the integral part and reset error slope
        if(COLDER(setpoint, prev_setpoint, 2) && HOTTER(ptc_cur_temp_k, setpoint, 2)){
            cooldown = true;
            heatup = false;
            error_integral = 0.0;
            error_slope= 0.0;
            error_integral = 1.5/I;
            // printf("cd! setpoint: %f, prev_setpoint: %f, limit: %f, current: %f\n", setpoint, prev_setpoint, prev_setpoint + (prev_setpoint/200.0), ptc_cur_temp_k);
        }
        else if (HOTTER(setpoint, prev_setpoint, 2) && COLDER(ptc_cur_temp_k, setpoint, 2)){
            heatup = true;
            cooldown = false;
            error_slope = 0.0;
            error_integral = error_integral < 1.5/I ? 1.5/I : error_integral;
            // printf("heatup! i: %f\n", error_integral);
        }
        if(cooldown == true && JUST_HOTTER(ptc_cur_temp_k, setpoint, 1)){
            cooldown = false;
            // printf("no cd!\n");
        }
        if(heatup == true && JUST_COLDER(ptc_cur_temp_k, setpoint, 1)){
            heatup = false;
            // printf("no heatup!\n");
        }
        if(cooldown == false && heatup == false){
            error_integral += (prev_error + temp_error) * (time_interval / 1000) / 2;
            if(error_integral < 0)
                error_integral = 0;
        }
        else if(heatup == true){
            // we still want a bit of I increase when heating up (/5 vs normal I coeff)
            error_integral += (prev_error + temp_error) / 5 * (time_interval / 1000) / 2;
        }
        if(ptc_cur_temp_k > CELSIUS_TO_KELVIN(200) && temp_target >= 235){
            // we need to increase P and I to reach the PTC highest temperatures
            float x = temp_target - 235;
            P = exp(x/15);
            I = exp(x/7) * 0.0035;
            D = 4/exp(x/15);
        }
        else{
            P = 1;
            I = 0.0035;
            D = 4;
        }

        duty_cycle = P * temp_error + I * error_integral + D * error_slope;
        duty_cycle = duty_cycle > 100 ? 100 : duty_cycle < 0 ? 0 : duty_cycle;

        xQueueSend(duty_cycle_queue, &duty_cycle, portMAX_DELAY);

        // update state for next loop iteration
        prev_error = temp_error;
        prev_setpoint = setpoint;
        i++;

        char * status = cooldown ? "-C" : heatup ? "+H" : "..";
        // printf("%u | duty: %.1f%%, P: %.1f(=%.1f), I: %.1f(=%.4f)(%s), D: %.1f(=%.1f) - temp_err: %.1f\n", i, duty_cycle, P*temp_error, P,  I*error_integral, I, status, D*error_slope, D, temp_error);
        printf("%.1f,%.1f,%.1f\n", duty_cycle, KELVIN_TO_CELSIUS(ptc_cur_temp_k), temp_target);

        vTaskDelay(delay);
    }
}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void ptc_control_task(void* unused_arg) {

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
        // xTaskCreate(ntc_pid_task,
        //     "ntc_pid_TASK",
        //     2048,
        //     NULL,
        //     1,
        //     &ntc_pid_task_handle),
        xTaskCreate(ptc_pid_task,
            "ptc_pid_TASK",
            2048,
            NULL,
            1,
            &ntc_pid_task_handle),
        xTaskCreate(ptc_control_task,
            "ptc_control_TASK",
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
