/**
 * @file hwt905_test.cpp
 * @author Dmitry Ponomarev
 */

#include <gtest/gtest.h>
#include <iostream>
#include "hwt905_driver.hpp"


void compare_arrays(const uint8_t first[], const uint8_t second[], size_t size) {
    // for (size_t idx = 0; idx < size; idx++) {
    //     std::cout << idx + 0 << ": " << first[idx] + 0 << " vs " << second[idx] + 0 << std::endl;
    // }
    for (size_t idx = 0; idx < size; idx++) {
        ASSERT_EQ(first[idx], second[idx]);
    }
}


/**
 * @brief linearize ring buffer
 */
TEST(Hwt905Test, linearize_ring_buffer_normal_normal){
    const uint8_t RING_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };
    const uint8_t EXCPECTED_LINEAR_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };
    uint8_t parsed_linear_buffer[Hwt905Driver::PAYLOAD_SIZE] = {};

    Hwt905Driver::linearize_ring_buffer(parsed_linear_buffer, RING_BUFFER, 10);
    compare_arrays(parsed_linear_buffer, EXCPECTED_LINEAR_BUFFER, Hwt905Driver::PAYLOAD_SIZE);
}

TEST(Hwt905Test, linearize_ring_buffer_offset_plus_1){
    const uint8_t RING_BUFFER[] = {
        11, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
    };
    const uint8_t EXCPECTED_LINEAR_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };

    uint8_t parsed_linear_buffer[Hwt905Driver::PAYLOAD_SIZE] = {};
    Hwt905Driver::linearize_ring_buffer(parsed_linear_buffer, RING_BUFFER, 0);
    compare_arrays(parsed_linear_buffer, EXCPECTED_LINEAR_BUFFER, Hwt905Driver::PAYLOAD_SIZE);
}

TEST(Hwt905Test, linearize_ring_buffer_offset_plus_2){
    const uint8_t RING_BUFFER[] = {
        10, 11, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    const uint8_t EXCPECTED_LINEAR_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };

    uint8_t parsed_linear_buffer[Hwt905Driver::PAYLOAD_SIZE] = {};
    Hwt905Driver::linearize_ring_buffer(parsed_linear_buffer, RING_BUFFER, 1);
    compare_arrays(parsed_linear_buffer, EXCPECTED_LINEAR_BUFFER, Hwt905Driver::PAYLOAD_SIZE);
}

TEST(Hwt905Test, linearize_ring_buffer_offset_minus_1){
    const uint8_t RING_BUFFER[] = {
        2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 1
    };
    const uint8_t EXCPECTED_LINEAR_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };

    uint8_t parsed_linear_buffer[Hwt905Driver::PAYLOAD_SIZE] = {};
    Hwt905Driver::linearize_ring_buffer(parsed_linear_buffer, RING_BUFFER, 9);
    compare_arrays(parsed_linear_buffer, EXCPECTED_LINEAR_BUFFER, Hwt905Driver::PAYLOAD_SIZE);
}

TEST(Hwt905Test, linearize_ring_buffer_offset_minus_2){
    const uint8_t RING_BUFFER[] = {
        3, 4, 5, 6, 7, 8, 9, 10, 11, 1, 2
    };
    const uint8_t EXCPECTED_LINEAR_BUFFER[Hwt905Driver::PAYLOAD_SIZE] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    };

    uint8_t parsed_linear_buffer[Hwt905Driver::PAYLOAD_SIZE] = {};
    Hwt905Driver::linearize_ring_buffer(parsed_linear_buffer, RING_BUFFER, 8);
    compare_arrays(parsed_linear_buffer, EXCPECTED_LINEAR_BUFFER, Hwt905Driver::PAYLOAD_SIZE);
}


/**
 * @brief process_next_byte()
 */
TEST(Hwt905Test, process_normal_single_real_msg){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x51, 0x01, 0x00, 0xFE, 0xFF, 0x00, 0x08, 0x7A, 0x0B, 0x31,
    };
    const uint8_t EXCPECTED_DATA_TYPES[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ACCEL,
    };

    uint8_t parsed_data_types[sizeof(EXCPECTED_DATA_TYPES)] = {};
    for (size_t idx = 0; idx < sizeof(EXCPECTED_DATA_TYPES); idx++) {
        parsed_data_types[idx] = hwt905_driver.process_next_byte(BUFFER[idx]);
    }
    compare_arrays(parsed_data_types, EXCPECTED_DATA_TYPES, sizeof(EXCPECTED_DATA_TYPES));
}

TEST(Hwt905Test, process_normal_few_real_msgs){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x51, 0x01, 0x00, 0xFE, 0xFF, 0x00, 0x08, 0x7A, 0x0B, 0x31,
        0x55, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A, 0x0B, 0x2C,
        0x55, 0x53, 0xF4, 0xFF, 0xF5, 0xFF, 0xD3, 0xC0, 0x24, 0x29, 0x6F,
        0x55, 0x54, 0x86, 0xF4, 0x37, 0x00, 0x92, 0xF8, 0x7A, 0x0B, 0x69,
        0x55, 0x51, 0x00, 0x00, 0xF9, 0xFF, 0x00, 0x08, 0x7F, 0x0B, 0x30,
        0x55, 0x52, 0x00, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0x7F, 0x0B, 0x30,
        0x55, 0x53, 0xF4, 0xFF, 0xF6, 0xFF, 0xD1, 0xC0, 0x24, 0x29, 0x6E,
    };
    const uint8_t EXCPECTED_DATA_TYPES[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ACCEL,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ANG_VEL,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ANGLE,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_MAG,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ACCEL,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ANG_VEL,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, DATA_TYPE_ANGLE,
    };

    uint8_t parsed_data_types[sizeof(EXCPECTED_DATA_TYPES)] = {};
    for (size_t idx = 0; idx < sizeof(EXCPECTED_DATA_TYPES); idx++) {
        parsed_data_types[idx] = hwt905_driver.process_next_byte(BUFFER[idx]);
    }
    compare_arrays(parsed_data_types, EXCPECTED_DATA_TYPES, sizeof(EXCPECTED_DATA_TYPES));
}


/**
 * @brief parse get_time()
 */
TEST(Hwt905Test, get_accel_normal){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x51, 0x01, 0x00, 0xFE, 0xFF, 0x00, 0x08, 0x7A, 0x0B, 0x31,
    };
    Hwt905_Acceleration_t expected_accel = {
        .ax = 0.0048,
        .ay = -0.0096,
        .az = 9.80,
        .temperature = 29.38,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        hwt905_driver.process_next_byte(BUFFER[idx]);
    }

    Hwt905_Acceleration_t actual_accel;
    hwt905_driver.get_acceleration(&actual_accel);
    ASSERT_NEAR(actual_accel.ax, expected_accel.ax, 0.001);
    ASSERT_NEAR(actual_accel.ay, expected_accel.ay, 0.001);
    ASSERT_NEAR(actual_accel.az, expected_accel.az, 0.001);
    ASSERT_NEAR(actual_accel.temperature, expected_accel.temperature, 0.01);
}


/**
 * @brief parse get_angular_velocity()
 */
TEST(Hwt905Test, get_angular_velocity_normal){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A, 0x0B, 0x2C,
    };
    Hwt905_AngularVelocity_t expected_ang_vel = {
        .wx = 0.0000,
        .wy = 0.0000,
        .wz = 0.0000,
        .temperature = 29.38,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        hwt905_driver.process_next_byte(BUFFER[idx]);
    }

    Hwt905_AngularVelocity_t actual_ang_vel;
    hwt905_driver.get_angular_velocity(&actual_ang_vel);
    ASSERT_NEAR(actual_ang_vel.wx, expected_ang_vel.wx, 0.001);
    ASSERT_NEAR(actual_ang_vel.wy, expected_ang_vel.wy, 0.001);
    ASSERT_NEAR(actual_ang_vel.wz, expected_ang_vel.wz, 0.001);
    ASSERT_NEAR(actual_ang_vel.temperature, expected_ang_vel.temperature, 0.01);
}


int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
