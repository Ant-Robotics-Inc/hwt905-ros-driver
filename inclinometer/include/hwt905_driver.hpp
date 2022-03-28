/**
 * @file hwt905_driver.hpp
 * @author Dmitry Ponomarev
 * @brief Based on https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Datasheet.pdf
 */

#ifndef HW905_DRIVER_HPP
#define HW905_DRIVER_HPP

#include <stdint.h>
#include <cstddef>


enum Hwt905_DataType_t {
    DATA_TYPE_NONE,
    DATA_TYPE_TIME,
    DATA_TYPE_ACCEL,
    DATA_TYPE_ANG_VEL,
    DATA_TYPE_ANGLE,
    DATA_TYPE_MAG,
    DATA_TYPE_QUATERNION,
    NUMBER_OF_DATA_TYPES,
};

struct Hwt905_Time_t {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t millisecond;
};

struct Hwt905_Acceleration_t {
    float ax;
    float ay;
    float az;
    float temperature;
};

struct Hwt905_AngularVelocity_t {
    float wx;
    float wy;
    float wz;
    float temperature;
};

struct Hwt905_Angle_t {
    float roll;
    float pitch;
    float yaw;
    uint16_t version;
};

struct Hwt905_Magnetic_t {
    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;
    float temperature;
};

struct Hwt905_Quaternion_t {
    float q_0;
    float q_1;
    float q_2;
    float q_3;
};


class Hwt905Driver {
public:
    Hwt905Driver() {};
    Hwt905_DataType_t process_next_byte(uint8_t byte);

    static constexpr const size_t PAYLOAD_SIZE = 11;
    static void linearize_ring_buffer(uint8_t linear_buf[PAYLOAD_SIZE],
                                      const uint8_t ring_buf[PAYLOAD_SIZE],
                                      size_t ring_buf_idx);

    bool get_time(Hwt905_Time_t* time);
    bool get_acceleration(Hwt905_Acceleration_t* accel);
    bool get_angular_velocity(Hwt905_AngularVelocity_t* ang_vel);
    bool get_angle(Hwt905_Angle_t* angle);
    bool get_magnetic_field(Hwt905_Magnetic_t* mag);
    bool get_quaternion(Hwt905_Quaternion_t* quaternion);
private:
    void _linearize_ring_buffer();
    Hwt905_DataType_t check_payload();

    bool is_check_sum_correct();


    uint8_t _ring_buffer[PAYLOAD_SIZE];
    uint8_t _linear_buffer[PAYLOAD_SIZE];
    size_t _ring_buffer_idx = 0;
};

#endif  // HW905_DRIVER_HPP