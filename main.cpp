#include <algorithm>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr gpio_num_t motor_pwm = GPIO_NUM_4;
static constexpr gpio_num_t encoder_a = GPIO_NUM_6;
static constexpr gpio_num_t encoder_b = GPIO_NUM_7;
static constexpr gpio_num_t boot_button = GPIO_NUM_0;

static constexpr int ENCODER_COUNTS_PER_REV = 80;
static constexpr int ENCODER_SIGN = 1;
static constexpr int step_counts = (ENCODER_COUNTS_PER_REV + 2) / 4;
static constexpr unsigned control_ms = 20;
static constexpr float dt = control_ms / 1000.0f;
static constexpr unsigned max_pwm = 450;
static constexpr float PID_KP = 22.0f;
static constexpr float PID_KI = 8.0f;
static constexpr float PID_KD = 0.85f;

static void init_pwm()
{
    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t motor{};
    motor.gpio_num = motor_pwm;
    motor.speed_mode = LEDC_LOW_SPEED_MODE;
    motor.channel = LEDC_CHANNEL_0;
    motor.timer_sel = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_channel_config(&motor));
}

static pcnt_unit_handle_t init_encoder()
{
    pcnt_unit_config_t limits{};
    limits.low_limit = -30000;
    limits.high_limit = 30000;
    limits.flags.accum_count = 1;

    pcnt_unit_handle_t unit;
    ESP_ERROR_CHECK(pcnt_new_unit(&limits, &unit));

    pcnt_glitch_filter_config_t filter{};
    filter.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

    pcnt_chan_config_t pins{};
    pins.edge_gpio_num = encoder_a;
    pins.level_gpio_num = encoder_b;
    pcnt_channel_handle_t a;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &pins, &a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pins.edge_gpio_num = encoder_b;
    pins.level_gpio_num = encoder_a;
    pcnt_channel_handle_t b;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &pins, &b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(gpio_set_pull_mode(encoder_a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(encoder_b, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, limits.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, limits.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

static unsigned pid_output(int error, float velocity, float &integral)
{
    const float ceiling = static_cast<float>(max_pwm);
    const float next_integral = std::clamp(integral + PID_KI * error * dt, 0.0f, ceiling);
    const float pd = PID_KP * error - PID_KD * velocity;
    if (pd + next_integral <= ceiling) {
        integral = next_integral;
    }
    return static_cast<unsigned>(std::clamp(pd + integral, 0.0f, ceiling));
}

extern "C" void app_main()
{
    init_pwm();
    const auto encoder = init_encoder();
    ESP_ERROR_CHECK(gpio_set_direction(boot_button, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(boot_button, GPIO_PULLUP_ONLY));

    int target = 0;
    int stable = 1;
    unsigned pending = 0;
    int previous = 0;
    int start_position = 0;
    unsigned move_ticks = 0;
    float velocity = 0;
    float integral = 0;
    bool moving = false;
    TickType_t next = xTaskGetTickCount();

    for (unsigned cycle = 0; ; ++cycle) {
        int count;
        ESP_ERROR_CHECK(pcnt_unit_get_count(encoder, &count));
        const int position = ENCODER_SIGN * count;
        const float measured_velocity = (position - previous) / dt;
        velocity += 0.35f * (measured_velocity - velocity);
        previous = position;

        const int key = gpio_get_level(boot_button);
        pending = key == stable ? 0 : pending + 1;
        if (pending >= 2) {
            stable = key;
            pending = 0;
            if (!key) {
                target += step_counts;
                if (!moving) {
                    start_position = position;
                    move_ticks = 0;
                }
                integral = 0;
                moving = true;
            }
        }

        const int error = target - position;
        unsigned pwm = 0;
        if (moving) {
            if (error <= 0) {
                moving = false;
            } else if (position < start_position - 3 || ++move_ticks >= 3000 / control_ms) {
                moving = false;
                target = position;
                std::puts("Motor stopped: check encoder direction or stalled motor");
            } else {
                pwm = pid_output(error, velocity, integral);
            }
        }

        ESP_ERROR_CHECK(ledc_set_duty_and_update(
            LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm, 0));

        if (cycle % 10 == 0) {
            std::printf("set=%d pos=%d error=%d pwm=%u\n", target, position, target - position, pwm);
        }
        xTaskDelayUntil(&next, pdMS_TO_TICKS(control_ms));
    }
}
