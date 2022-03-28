/**
 * @file hwt905_driver.cpp
 * @author ponomarevda96@gmail.com
 */

#include <hwt905_driver.hpp>
#include <ros/ros.h>


#define START_BYTE              0x55
#define START_BYTE_TIME         0x50
#define START_BYTE_ACCEL        0x51
#define START_BYTE_ANG_VEL      0x52
#define START_BYTE_ANGLE        0x53
#define START_BYTE_MAG          0x54
#define START_BYTE_QUATERNION   0x59


enum DataTypeHeader_t : uint16_t {
    HEADER_TIME         = (START_BYTE) | (START_BYTE_TIME << 8),
    HEADER_ACCEL        = (START_BYTE) | (START_BYTE_ACCEL << 8),
    HEADER_ANG_VEL      = (START_BYTE) | (START_BYTE_ANG_VEL << 8),
    HEADER_ANGLE        = (START_BYTE) | (START_BYTE_ANGLE << 8),
    HEADER_MAG          = (START_BYTE) | (START_BYTE_MAG << 8),
    HEADER_QUATERNION   = (START_BYTE) | (START_BYTE_QUATERNION << 8),
};

///< 5.1.1 Time Output
#pragma pack(push, 1)
struct TimePayload_t {
    uint16_t header;
    uint8_t YY;
    uint8_t MM;
    uint8_t DD;
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint16_t ms;
    uint8_t SUM;
};

///< 5.1.2 Acceleration Output
struct AccelerationPayload_t {
    uint16_t header;
    int16_t ax;
    int16_t ay;
    int16_t az;
    uint16_t temperature;
    uint8_t SUM;
};

///< 5.1.3 Angular Velocity Output
struct AngularVelocityPayload_t {
    uint16_t header;
    int16_t wx;
    int16_t wy;
    int16_t wz;
    uint16_t temperature;
    uint8_t sum;
};

///< 5.1.4  Angle Output
struct AnglePayload_t {
    uint16_t header;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t version;
    uint8_t sum;
};

///< 5.1.5 Magnetic Output
struct MagneticPayload_t {
    uint16_t header;
    int16_t Hx;
    int16_t Hy;
    int16_t Hz;
    uint16_t temperature;
    uint8_t sum;
};

///< 5.1.6 Quaternion Output
struct QuaternionPayload_t {
    uint16_t header;
    int16_t q[4];
    uint8_t sum;
};
#pragma pack(pop)


constexpr const size_t Hwt905Driver::PAYLOAD_SIZE;


// All structures must be packed.
static_assert(sizeof(TimePayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(AccelerationPayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(AngularVelocityPayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(AnglePayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(MagneticPayload_t) == Hwt905Driver::PAYLOAD_SIZE);
static_assert(sizeof(QuaternionPayload_t) == Hwt905Driver::PAYLOAD_SIZE);


Hwt905_DataType_t Hwt905Driver::process_next_byte(uint8_t byte) {
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

bool Hwt905Driver::get_time(Hwt905_Time_t* time) {
    if (time == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const TimePayload_t*>(_linear_buffer);
    time->year = payload->YY;
    time->month = payload->MM;
    time->day = payload->DD;
    time->hour = payload->hh;
    time->minute = payload->mm;
    time->second = payload->ss;
    time->millisecond = payload->ms;

    return true;
}

bool Hwt905Driver::get_acceleration(Hwt905_Acceleration_t* accel) {
    if (accel == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const AccelerationPayload_t*>(_linear_buffer);
    accel->ax = payload->ax * (9.8 * 16 / 32768);
    accel->ay = payload->ay * (9.8 * 16 / 32768);
    accel->az = payload->az * (9.8 * 16 / 32768);
    accel->temperature = payload->temperature * 0.01;

    return true;
}

bool Hwt905Driver::get_angular_velocity(Hwt905_AngularVelocity_t* ang_vel) {
    if (ang_vel == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const AngularVelocityPayload_t*>(_linear_buffer);
    ang_vel->wx = payload->wx * (2000.0 / 32768);
    ang_vel->wy = payload->wy * (2000.0 / 32768);
    ang_vel->wz = payload->wz * (2000.0 / 32768);
    ang_vel->temperature = payload->temperature * 0.01;

    return true;
}

bool Hwt905Driver::get_angle(Hwt905_Angle_t* angle) {
    if (angle == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const AnglePayload_t*>(_linear_buffer);
    angle->roll = payload->roll * (180.0 / 32768);
    angle->pitch = payload->pitch * (180.0 / 32768);
    angle->yaw = payload->yaw * (180.0 / 32768);
    angle->version = payload->version;

    return true;
}

bool Hwt905Driver::get_magnetic_field(Hwt905_Magnetic_t* mag) {
    if (mag == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const MagneticPayload_t*>(_linear_buffer);
    mag->mag_x = payload->Hx;
    mag->mag_y = payload->Hy;
    mag->mag_z = payload->Hz;
    mag->temperature = payload->temperature * 0.01;

    return true;
}

bool Hwt905Driver::get_quaternion(Hwt905_Quaternion_t* quaternion) {
    if (quaternion == nullptr) {
        return false;
    }

    auto payload = reinterpret_cast<const QuaternionPayload_t*>(_linear_buffer);
    quaternion->q_0 = payload->q[0] / 32768.0;
    quaternion->q_1 = payload->q[1] / 32768.0;
    quaternion->q_2 = payload->q[2] / 32768.0;
    quaternion->q_3 = payload->q[3] / 32768.0;

    return true;
}


void Hwt905Driver::_linearize_ring_buffer() {
    linearize_ring_buffer(_linear_buffer, _ring_buffer, _ring_buffer_idx);
}

Hwt905_DataType_t Hwt905Driver::check_payload() {
    if (!is_check_sum_correct()) {
        return DATA_TYPE_NONE;
    }

    auto header = reinterpret_cast<const DataTypeHeader_t*>(_linear_buffer)[0];
    Hwt905_DataType_t data_type;
    switch (header) {
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


bool Hwt905Driver::is_check_sum_correct() {
    auto payload = reinterpret_cast<const AccelerationPayload_t*>(_linear_buffer);
    uint8_t calculated_sum = 0;
    for (size_t idx = 0; idx < 10; idx++) {
        calculated_sum += _linear_buffer[idx];
    }
    uint8_t received_sum = _linear_buffer[10];
    return (calculated_sum == received_sum) ? true : false;
}
