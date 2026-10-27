/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected Xsens MTi AHRS systems
 */

#include "AP_ExternalAHRS_config.h"

#if HAL_EXTERNAL_AHRS_ENABLED && AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_Xsens.h"

#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <cstring>

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint8_t XSENS_PREAMBLE = 0xFA;
constexpr uint8_t XSENS_BUS_ID = 0xFF;
constexpr uint8_t XSENS_MT_DATA2 = 0x36;
constexpr uint16_t XSENS_DATA_ID_PACKET_COUNTER = 0x1020;
constexpr uint16_t XSENS_DATA_ID_SAMPLE_TIME_FINE = 0x1060;
constexpr uint16_t XSENS_DATA_ID_QUATERNION = 0x2010;
constexpr uint16_t XSENS_DATA_ID_BAROMETRIC_PRESSURE = 0x3010;
constexpr uint16_t XSENS_DATA_ID_ACCELERATION = 0x4020;
constexpr uint16_t XSENS_DATA_ID_ANGULAR_RATE = 0x8020;
constexpr uint16_t XSENS_DATA_ID_MAGNETIC_FIELD = 0xC020;
constexpr uint16_t XSENS_DATA_ID_STATUS_WORD = 0xE020;

}  // namespace

AP_ExternalAHRS_Xsens::AP_ExternalAHRS_Xsens(AP_ExternalAHRS *_frontend,
                                             AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens ExternalAHRS: no UART");
        return;
    }

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    uart->begin(baudrate);
    uart_started = true;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens ExternalAHRS initialised");
}

int8_t AP_ExternalAHRS_Xsens::get_port() const
{
    if (uart == nullptr) {
        return -1;
    }
    return port_num;
}

const char *AP_ExternalAHRS_Xsens::get_name() const
{
    return "Xsens";
}

bool AP_ExternalAHRS_Xsens::healthy() const
{
    const uint32_t now_ms = AP_HAL::millis();
    return (received_quaternion && now_ms - last_quaternion_ms < 250) ||
           (received_accel && now_ms - last_accel_ms < 500) ||
           (received_gyro && now_ms - last_gyro_ms < 500);
}

bool AP_ExternalAHRS_Xsens::initialised() const
{
    return received_quaternion || received_accel || received_gyro;
}

bool AP_ExternalAHRS_Xsens::pre_arm_check(char *failure_msg,
                                          uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_Xsens::get_filter_status(nav_filter_status &status) const
{
    status = {};
    status.flags.initalized = initialised();
    status.flags.attitude = received_quaternion;
    status.flags.vert_vel = true;
    status.flags.vert_pos = true;
}

void AP_ExternalAHRS_Xsens::update()
{
    if (uart == nullptr) {
        return;
    }

    if (!uart_started) {
        uart->begin(baudrate);
        uart_started = true;
    }

    const uint32_t nbytes = uart->available();
    if (nbytes > 0) {
        const uint32_t copy_len = MIN(nbytes, uint32_t(BUFFER_SIZE - buffer_used));
        if (copy_len == 0) {
            // buffer full; drop oldest data and keep parsing latest bytes
            memmove(buffer, &buffer[1], BUFFER_SIZE - 1);
            buffer_used = BUFFER_SIZE - 1;
        }

        if (copy_len > 0) {
            const ssize_t nread = uart->read(&buffer[buffer_used], copy_len);
            if (nread > 0) {
                buffer_used += uint16_t(nread);
            }
        }

        parse_buffer();
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_attitude_text_ms >= 1000) {
        if (received_quaternion && now_ms - last_quaternion_ms < 250) {
            Quaternion attitude;
            {
                WITH_SEMAPHORE(state.sem);
                attitude = state.quat;
            }
            float roll;
            float pitch;
            float yaw;
            attitude.to_euler(roll, pitch, yaw);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "XSens Roll:%0.1f Pitch:%0.1f Yaw:%0.1f",
                          degrees(roll), degrees(pitch), degrees(yaw));
        }
        last_attitude_text_ms = now_ms;
    }
}

static uint16_t read_u16_be(const uint8_t *data);

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t(data[0]) << 8) | uint16_t(data[1]);
}

void AP_ExternalAHRS_Xsens::parse_buffer()
{
    while (buffer_used >= 5) {
        if (buffer[0] != XSENS_PREAMBLE || buffer[1] != XSENS_BUS_ID) {
            memmove(buffer, &buffer[1], buffer_used - 1);
            buffer_used--;
            continue;
        }

        uint16_t payload_length = buffer[3];
        uint16_t header_length = 4;
        if (payload_length == 0xFF) {
            if (buffer_used < 7) {
                break;
            }
            payload_length = (uint16_t(buffer[4]) << 8) | buffer[5];
            header_length = 6;
        }

        if (payload_length == 0 || payload_length > (BUFFER_SIZE - header_length - 1)) {
            memmove(buffer, &buffer[1], buffer_used - 1);
            buffer_used--;
            continue;
        }

        const uint16_t total_length = header_length + payload_length + 1;
        if (buffer_used < total_length) {
            break;
        }

        uint8_t checksum = 0;
        for (uint16_t i = 1; i < total_length; i++) {
            checksum += buffer[i];
        }
        if (checksum != 0) {
            memmove(buffer, &buffer[1], buffer_used - 1);
            buffer_used--;
            continue;
        }

        if (buffer[2] == XSENS_MT_DATA2) {
            process_mtdata2(&buffer[header_length], payload_length);
        }

        memmove(buffer, &buffer[total_length], buffer_used - total_length);
        buffer_used -= total_length;
    }
}

bool AP_ExternalAHRS_Xsens::process_mtdata2(const uint8_t *payload,
                                            uint16_t payload_length)
{
    uint16_t offset = 0;
    AP_ExternalAHRS::ins_data_message_t ins_data {};
    bool packet_has_accel = false;
    bool packet_has_gyro = false;

    while (offset < payload_length) {
        // MTData2 item format: DataID(2) + Size(1) + Data(n)
        if (payload_length - offset < 3) {
            return false;
        }

        const uint16_t data_id = read_u16_be(&payload[offset]);
        const uint8_t data_length = payload[offset + 2];
        offset += 3;

        if (payload_length - offset < data_length) {
            return false;
        }

        const uint8_t *data = &payload[offset];
        if (data_id == XSENS_DATA_ID_QUATERNION && data_length >= 16) {
            const float q1 = read_float_be(data + 0);
            const float q2 = read_float_be(data + 4);
            const float q3 = read_float_be(data + 8);
            const float q4 = read_float_be(data + 12);
            if (!isfinite(q1) || !isfinite(q2) || !isfinite(q3) || !isfinite(q4)) {
                return false;
            }
            const Quaternion quat(q1, q2, q3, q4);
            if (quat.is_nan() || quat.is_zero()) {
                return false;
            }

            // Xsens reports sensor (X fwd, Y left, Z up) to ENU; ArduPilot needs FRD to NED.
            const float r = M_SQRT1_2;
            const Quaternion q_ned_enu(0, r, r, 0);
            const Quaternion q_flu_frd(0, 1, 0, 0);
            Quaternion quat_ned = q_ned_enu * quat * q_flu_frd;
            quat_ned.normalize();
            WITH_SEMAPHORE(state.sem);
            state.quat = quat_ned;
            state.have_quaternion = true;
            received_quaternion = true;
            last_quaternion_ms = AP_HAL::millis();
        } else if ((data_id == XSENS_DATA_ID_ACCELERATION ||
                    data_id == XSENS_DATA_ID_ANGULAR_RATE) && data_length >= 12) {
            const float x = read_float_be(data + 0);
            const float y = read_float_be(data + 4);
            const float z = read_float_be(data + 8);
            if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
                return false;
            }

            // FLU to FRD
            const Vector3f frd(x, -y, -z);
            WITH_SEMAPHORE(state.sem);
            if (data_id == XSENS_DATA_ID_ACCELERATION) {
                state.accel = frd;
                ins_data.accel = frd;
                packet_has_accel = true;
                received_accel = true;
                last_accel_ms = AP_HAL::millis();
            } else {
                state.gyro = frd;
                ins_data.gyro = frd;
                packet_has_gyro = true;
                received_gyro = true;
                last_gyro_ms = AP_HAL::millis();
            }
        } else if (data_id == XSENS_DATA_ID_MAGNETIC_FIELD && data_length >= 12) {
            const float mx = read_float_be(data + 0);
            const float my = read_float_be(data + 4);
            const float mz = read_float_be(data + 8);
            if (!isfinite(mx) || !isfinite(my) || !isfinite(mz)) {
                return false;
            }

            AP_ExternalAHRS::mag_data_message_t mag;
            mag.field = Vector3f(mx, -my, -mz) * 1.0e7f;
            AP::compass().handle_external(mag);
        } else if (data_id == XSENS_DATA_ID_BAROMETRIC_PRESSURE && data_length >= 4) {
            const float pressure_pa = read_float_be(data) * 100.0f;
            if (!isfinite(pressure_pa) || pressure_pa < 10000 || pressure_pa > 120000) {
                return false;
            }

            AP_ExternalAHRS::baro_data_message_t baro;
            baro.instance = 0;
            baro.pressure_pa = pressure_pa;
            baro.temperature = 25.0f;
            AP::baro().handle_external(baro);
        }

        // Other MTData2 items are intentionally ignored.

        offset += data_length;
    }

    if (packet_has_accel && packet_has_gyro) {
        ins_data.temperature = -300;
        AP::ins().handle_external(ins_data);
    }

    return true;
}

float AP_ExternalAHRS_Xsens::read_float_be(const uint8_t *data)
{
    uint32_t raw = (uint32_t(data[0]) << 24) |
                   (uint32_t(data[1]) << 16) |
                   (uint32_t(data[2]) << 8) |
                   uint32_t(data[3]);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED && AP_EXTERNAL_AHRS_XSENS_ENABLED
