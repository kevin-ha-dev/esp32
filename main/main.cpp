#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "config.h"
#include "motors.h"

// TEST ONLY: IMU pitch drives motor forward/reverse (not real balancing).

static constexpr int kI2cTimeoutMs = 200;
static constexpr size_t kMaxPacket = 512;

static constexpr uint8_t kChExecutable = 1;
static constexpr uint8_t kChControl = 2;
static constexpr uint8_t kChReports = 3;

static constexpr uint8_t kReportSetFeature = 0xFD;
static constexpr uint8_t kReportProductIdReq = 0xF9;
static constexpr uint8_t kReportProductIdResp = 0xF8;
static constexpr uint8_t kSensorGameRotationVector = 0x08;

static constexpr float kDeadzoneDeg = 5.0f;
static constexpr float kFullTiltDeg = 30.0f;
static constexpr float kMaxSpeed = 0.45f;

static i2c_master_dev_handle_t s_dev = nullptr;
static uint8_t s_seq[6] = {};
static uint8_t s_packet[kMaxPacket] = {};
static uint16_t s_packet_len = 0;
static uint8_t s_channel = 0;

static bool i2c_read(uint8_t *data, size_t len)
{
    return i2c_master_receive(s_dev, data, len, kI2cTimeoutMs) == ESP_OK;
}

static bool i2c_write(const uint8_t *data, size_t len)
{
    return i2c_master_transmit(s_dev, data, len, kI2cTimeoutMs) == ESP_OK;
}

static bool receive_packet(void)
{
    uint8_t header[4] = {};
    if (!i2c_read(header, 4)) {
        return false;
    }

    uint16_t packet_len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_len &= 0x7FFF;
    if (packet_len < 4 || packet_len > kMaxPacket) {
        return false;
    }

    if (!i2c_read(s_packet, packet_len)) {
        return false;
    }

    s_packet_len = packet_len;
    s_channel = s_packet[2];
    return true;
}

static bool send_packet(uint8_t channel, const uint8_t *data, uint16_t len)
{
    uint8_t buf[64] = {};
    uint16_t packet_len = len + 4;
    if (packet_len > sizeof(buf)) {
        return false;
    }

    buf[0] = packet_len & 0xFF;
    buf[1] = (packet_len >> 8) & 0xFF;
    buf[2] = channel;
    buf[3] = s_seq[channel]++;
    memcpy(buf + 4, data, len);
    return i2c_write(buf, packet_len);
}

static bool soft_reset(void)
{
    memset(s_seq, 0, sizeof(s_seq));
    uint8_t cmd = 1;
    return send_packet(kChExecutable, &cmd, 1);
}

static bool request_product_id(void)
{
    uint8_t cmd[2] = {kReportProductIdReq, 0};
    return send_packet(kChControl, cmd, sizeof(cmd));
}

static bool enable_game_rv(uint32_t interval_us)
{
    uint8_t cmd[17] = {};
    cmd[0] = kReportSetFeature;
    cmd[1] = kSensorGameRotationVector;
    cmd[5] = interval_us & 0xFF;
    cmd[6] = (interval_us >> 8) & 0xFF;
    cmd[7] = (interval_us >> 16) & 0xFF;
    cmd[8] = (interval_us >> 24) & 0xFF;
    return send_packet(kChControl, cmd, sizeof(cmd));
}

static void quat_to_roll_pitch(float i, float j, float k, float real, float *roll_deg, float *pitch_deg)
{
    float sinr_cosp = 2.0f * (real * i + j * k);
    float cosr_cosp = 1.0f - 2.0f * (i * i + j * j);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    float sinp = 2.0f * (real * j - k * i);
    if (fabsf(sinp) >= 1.0f) {
        *pitch_deg = copysignf(90.0f, sinp);
    } else {
        *pitch_deg = asinf(sinp) * 57.2957795f;
    }
}

static bool parse_game_rv(float *roll, float *pitch)
{
    if (s_channel != kChReports || s_packet_len < 4) {
        return false;
    }

    const uint8_t *cargo = s_packet + 4;
    const uint16_t cargo_len = s_packet_len - 4;

    for (uint16_t i = 0; i + 12 <= cargo_len; i++) {
        if (cargo[i] != kSensorGameRotationVector) {
            continue;
        }
        int16_t qi = (int16_t)((cargo[i + 5] << 8) | cargo[i + 4]);
        int16_t qj = (int16_t)((cargo[i + 7] << 8) | cargo[i + 6]);
        int16_t qk = (int16_t)((cargo[i + 9] << 8) | cargo[i + 8]);
        int16_t qr = (int16_t)((cargo[i + 11] << 8) | cargo[i + 10]);
        const float scale = 1.0f / 16384.0f;
        quat_to_roll_pitch(qi * scale, qj * scale, qk * scale, qr * scale, roll, pitch);
        return true;
    }
    return false;
}

static float pitch_to_motor(float pitch_deg)
{
    if (fabsf(pitch_deg) < kDeadzoneDeg) {
        return 0.0f;
    }

    float signed_mag = pitch_deg;
    if (signed_mag > kFullTiltDeg) {
        signed_mag = kFullTiltDeg;
    } else if (signed_mag < -kFullTiltDeg) {
        signed_mag = -kFullTiltDeg;
    }

    return (signed_mag / kFullTiltDeg) * kMaxSpeed;
}

extern "C" void app_main(void)
{
    printf("\nTEST: IMU pitch drives motor forward/reverse\n");
    printf("Lift the wheel. LiPo ON. Flat=stop, tip=spin.\n");
    printf("If direction feels backwards, we can flip the sign.\n\n");

    motor_init();
    motor_stop();

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = IMU_SDA_GPIO;
    bus_cfg.scl_io_num = IMU_SCL_GPIO;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = IMU_I2C_ADDR;
    dev_cfg.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev));

    printf("I2C: SDA=%d SCL=%d addr=0x%02X\n", IMU_SDA_GPIO, IMU_SCL_GPIO, IMU_I2C_ADDR);

    vTaskDelay(pdMS_TO_TICKS(300));
    soft_reset();
    printf("Soft reset sent\n");
    vTaskDelay(pdMS_TO_TICKS(300));
    memset(s_seq, 0, sizeof(s_seq));

    bool reset_done = false;
    for (int i = 0; i < 50 && !reset_done; i++) {
        if (receive_packet()) {
            if (s_channel == kChExecutable && s_packet_len >= 5 && s_packet[4] == 1) {
                printf("Reset complete\n");
                reset_done = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    request_product_id();
    bool got_pid = false;
    for (int i = 0; i < 40; i++) {
        if (receive_packet()) {
            for (uint16_t b = 4; b < s_packet_len; b++) {
                if (s_packet[b] == kReportProductIdResp) {
                    printf("Product ID OK\n");
                    got_pid = true;
                    break;
                }
            }
            if (got_pid) {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!got_pid) {
        printf("No product ID response (continuing)\n");
    }

    enable_game_rv(10000);
    printf("Ready. Tip forward/back to spin the wheel.\n");

    int print_div = 0;
    while (1) {
        if (receive_packet()) {
            float roll = 0, pitch = 0;
            if (parse_game_rv(&roll, &pitch)) {
                float speed = pitch_to_motor(pitch);
                motor_set(speed);

                if (++print_div >= 10) {
                    print_div = 0;
                    const char *dir =
                        (speed > 0.02f) ? "FWD" : (speed < -0.02f) ? "REV" : "STOP";
                    printf("pitch=%6.1f  motor=%5.2f  %s\n", pitch, speed, dir);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
