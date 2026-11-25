#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_err.h"

#define MOTOR_PWM_GPIO      4
#define ENCODER_GPIO_A       5
#define ENCODER_GPIO_B       6
#define BUTTON_GPIO           GPIO_NUM_0

#define LEDC_DUTY_BITS       LEDC_TIMER_10_BIT
#define LEDC_PWM_FREQ_HZ     5000
#define PWM_MAX_DUTY         450
#define PID_KP               4.9f
#define PID_KI               1.8f
#define PID_KD               0.19f
#define ENCODER_SIGN         1

#define ENCODER_PPR          20
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4)

#define PID_DT_S              0.01f
#define PID_PERIOD_US         10000

typedef struct {
    float kp, ki, kd;
    float dt;
    float integral, integral_min, integral_max;
    float prev_error;
    float d_filtered;
    float d_alpha;
    bool  enable_i;
    bool  enable_d;
} MotorPid;

static void pid_init(MotorPid *p, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->dt = dt;
    p->integral = 0.0f;
    p->integral_min = out_min;
    p->integral_max = ki > 0.0f ? out_max / ki : 0.0f;
    p->prev_error = 0.0f;
    p->d_filtered = 0.0f;
    p->d_alpha = 0.8f;
    p->enable_i = true;
    p->enable_d = true;
}

static float pid_compute(MotorPid *p, float setpoint, float measurement) {
    float error = setpoint - measurement;

    float p_term = p->kp * error;

    float i_term = 0.0f;
    if (p->enable_i) {
        i_term = p->ki * p->integral;
    }

    float d_term = 0.0f;
    if (p->enable_d) {
        float d_raw = (error - p->prev_error) / p->dt;
        p->d_filtered = p->d_alpha * p->d_filtered + (1.0f - p->d_alpha) * d_raw;
        d_term = p->kd * p->d_filtered;
    }

    if (p->enable_i && p_term + i_term + d_term < PWM_MAX_DUTY) {
        p->integral += error * p->dt;
        if (p->integral > p->integral_max) p->integral = p->integral_max;
        if (p->integral < p->integral_min) p->integral = p->integral_min;
        i_term = p->ki * p->integral;
    }

    p->prev_error = error;
    return p_term + i_term + d_term;
}

static void motor_pwm_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = LEDC_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t chan_cfg = {
        .gpio_num   = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));
}

static unsigned motor_set_duty(float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_MAX_DUTY) duty = (float)PWM_MAX_DUTY;
    const unsigned pwm = (unsigned)duty;
    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm, 0));
    return pwm;
}

static pcnt_unit_handle_t pcnt_unit = NULL;

static void encoder_init(void) {
    pcnt_unit_config_t unit_config = {
        .low_limit = -30000,
        .high_limit = 30000,
        .flags = {.accum_count = 1},
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)ENCODER_GPIO_A, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)ENCODER_GPIO_B, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

static float encoder_get_angle_deg(void) {
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));
    return (float)(ENCODER_SIGN * count) * (360.0f / (float)ENCODER_COUNTS_PER_REV);
}

static float setpoint_deg = 0.0f;

static void button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static bool check_button_and_advance_setpoint(void) {
    static bool stable = false;
    static unsigned pending = 0;
    const bool pressed = gpio_get_level(BUTTON_GPIO) == 0;
    pending = pressed == stable ? 0 : pending + 1;
    if (pending < 5) {
        return false;
    }

    stable = pressed;
    pending = 0;
    if (stable) {
        setpoint_deg += 90.0f;
    }
    return stable;
}

static TaskHandle_t pid_task_handle = NULL;

static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t,
                                      const gptimer_alarm_event_data_t *,
                                      void *) {
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(pid_task_handle, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void pid_timer_init(void) {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src     = GPTIMER_CLK_SRC_DEFAULT,
        .direction   = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = PID_PERIOD_US,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

extern "C" void app_main(void) {
    pid_task_handle = xTaskGetCurrentTaskHandle();

    motor_pwm_init();
    encoder_init();
    button_init();

    MotorPid motor_pid;
    pid_init(&motor_pid, PID_KP, PID_KI, PID_KD,
             PID_DT_S, 0.0f, (float)PWM_MAX_DUTY);
    pid_timer_init();

    int64_t started_us = 0;
    float start_angle = 0.0f;
    bool moving = false;
    unsigned cycle = 0;

    printf("t_ms,setpoint,pv,error,duty\n");

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const float pv = encoder_get_angle_deg();
        const int64_t now_us = esp_timer_get_time();
        if (check_button_and_advance_setpoint()) {
            if (!moving) {
                started_us = now_us;
                start_angle = pv;
            }
            moving = true;
            motor_pid.integral = 0.0f;
            motor_pid.prev_error = setpoint_deg - pv;
            motor_pid.d_filtered = 0.0f;
        }

        if (moving && pv >= setpoint_deg) {
            moving = false;
        }
        if (moving && (pv < start_angle - 13.5f || now_us - started_us >= 3000000)) {
            moving = false;
            setpoint_deg = pv;
            puts("Motor stopped: check encoder direction or stalled motor");
        }

        const float output = moving ? pid_compute(&motor_pid, setpoint_deg, pv) : 0.0f;
        const unsigned pwm = motor_set_duty(output);
        if (cycle++ % 10 == 0) {
            printf("%lld,%.1f,%.1f,%.1f,%u\n",
                   (long long)(now_us / 1000), setpoint_deg, pv, setpoint_deg - pv, pwm);
        }
    }
}
