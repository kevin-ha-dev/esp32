#include "motors.h"
#include "config.h"

#include <algorithm>
#include "driver/ledc.h"

static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
static constexpr ledc_channel_t kChIn1 = LEDC_CHANNEL_0;
static constexpr ledc_channel_t kChIn2 = LEDC_CHANNEL_1;

static void set_duty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(kSpeedMode, channel, duty);
    ledc_update_duty(kSpeedMode, channel);
}

void motor_init(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kSpeedMode;
    timer.duty_resolution = MOTOR_PWM_RESOLUTION;
    timer.timer_num = kTimer;
    timer.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {};
    ch.speed_mode = kSpeedMode;
    ch.timer_sel = kTimer;
    ch.duty = 0;
    ch.hpoint = 0;

    ch.channel = kChIn1;
    ch.gpio_num = MOTOR_IN1;
    ledc_channel_config(&ch);

    ch.channel = kChIn2;
    ch.gpio_num = MOTOR_IN2;
    ledc_channel_config(&ch);

    motor_stop();
}

void motor_set(float speed)
{
    speed = std::clamp(speed, -1.0f, 1.0f);
    const float mag = (speed < 0.0f) ? -speed : speed;
    // Slow-decay PWM: one pin held on, the other PWM'd.
    // Off-time brakes instead of coasts, so low speed has more torque.
    const uint32_t drive = MOTOR_PWM_MAX;
    const uint32_t brake = static_cast<uint32_t>((1.0f - mag) * MOTOR_PWM_MAX);

    if (speed > 0.0f) {
        set_duty(kChIn1, drive);
        set_duty(kChIn2, brake);
    } else if (speed < 0.0f) {
        set_duty(kChIn2, drive);
        set_duty(kChIn1, brake);
    } else {
        motor_stop();
    }
}

void motor_stop(void)
{
    set_duty(kChIn1, 0);
    set_duty(kChIn2, 0);
}
