#pragma once

#include "AP_ExternalAHRS_config.h"

#if HAL_EXTERNAL_AHRS_ENABLED && AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_Xsens : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_Xsens(
        AP_ExternalAHRS *frontend,
        AP_ExternalAHRS::state_t &state);

    int8_t get_port() const override;
    const char *get_name() const override;

    bool healthy() const override;
    bool initialised() const override;

    bool pre_arm_check(
        char *failure_msg,
        uint8_t failure_msg_len) const override;

    void get_filter_status(
        nav_filter_status &status) const override;

    void update() override;

private:
    AP_HAL::UARTDriver *uart = nullptr;
    int8_t port_num = -1;
    uint32_t baudrate = 0;

    // UART initialization is performed in update().
    bool uart_started = false;

    // Bounded Xbus receive buffer.
    static constexpr uint16_t BUFFER_SIZE = 512;
    uint8_t buffer[BUFFER_SIZE] {};
    uint16_t buffer_used = 0;

    // Track each output independently.
    bool received_quaternion = false;
    bool received_accel = false;
    bool received_gyro = false;

    uint32_t last_quaternion_ms = 0;
    uint32_t last_accel_ms = 0;
    uint32_t last_gyro_ms = 0;
    uint32_t last_attitude_text_ms = 0;

    void check_uart();
    void parse_buffer();

    bool process_mtdata2(
        const uint8_t *payload,
        uint16_t payload_length);

    static float read_float_be(const uint8_t *data);
};

#endif