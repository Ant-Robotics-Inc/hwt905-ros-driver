/**
 * @file hwt905_driver.cpp
 * @author ponomarevda96@gmail.com
 */

#include <hwt905_driver.hpp>
#include <ros/ros.h>


constexpr const size_t Hwt905Driver::PAYLOAD_SIZE;


// All structures must be packed.
static_assert(sizeof(TimePayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(Acceleration_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(AngularVelocity_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(Angle_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(Magnetic_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(Quaternion_t) == Hwt905Driver::PAYLOAD_SIZE);


DataType_t Hwt905Driver::process(uint8_t byte) {
    _ring_buffer[_ring_buffer_idx] = byte;    
    _linearize_ring_buffer();
    auto data_type = check_payload();

    _ring_buffer_idx = _ring_buffer_idx < PAYLOAD_SIZE - 1 ? _ring_buffer_idx + 1 : 0;

    return data_type;
}

void Hwt905Driver::linearize_ring_buffer(uint8_t linear_buf[PAYLOAD_SIZE],
                                         const uint8_t ring_buf[PAYLOAD_SIZE],
                                         size_t last_written_idx) {
    size_t beggining_idx = last_written_idx + 1;
    if (beggining_idx >= PAYLOAD_SIZE) {
        beggining_idx = 0;
    }

    memcpy(linear_buf, ring_buf + beggining_idx, PAYLOAD_SIZE - beggining_idx);
    memcpy(linear_buf + PAYLOAD_SIZE - beggining_idx, ring_buf, beggining_idx);
}

void Hwt905Driver::_linearize_ring_buffer() {
    linearize_ring_buffer(_linear_buffer, _ring_buffer, _ring_buffer_idx);
}

DataType_t Hwt905Driver::check_payload() {
    auto header = reinterpret_cast<const DataTypeHeader_t*>(_linear_buffer)[0];
    auto data_type = DATA_TYPE_NONE;
    switch (header)
    {
    case HEADER_TIME:
        data_type = DATA_TYPE_TIME;
        break;
    
    case HEADER_ACCEL:
        data_type = DATA_TYPE_ACCEL;
        break;

    case HEADER_ANG_VEL:
        data_type = DATA_TYPE_ANG_VEL;
        break;

    case HEADER_ANGLE:
        data_type = DATA_TYPE_ANGLE;
        break;

    case HEADER_MAG:
        data_type = DATA_TYPE_MAG;
        break;

    case HEADER_QUATERNION:
        data_type = DATA_TYPE_QUATERNION;
        break;

    default:
        data_type = DATA_TYPE_NONE;
    }

    return data_type;
}
