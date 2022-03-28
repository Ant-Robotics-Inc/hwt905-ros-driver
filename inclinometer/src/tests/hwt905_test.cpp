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
 * @brief parse get_accel()
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

    for (size_t idx = 0; idx < sizeof(BUFFER) - 1; idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
    ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[10]), DATA_TYPE_ACCEL);

    Hwt905_Acceleration_t actual_accel;
    hwt905_driver.get_acceleration(&actual_accel);
    ASSERT_NEAR(actual_accel.ax, expected_accel.ax, 0.001);
    ASSERT_NEAR(actual_accel.ay, expected_accel.ay, 0.001);
    ASSERT_NEAR(actual_accel.az, expected_accel.az, 0.001);
    ASSERT_NEAR(actual_accel.temperature, expected_accel.temperature, 0.01);
}

TEST(Hwt905Test, get_accel_wrong_checksum){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x51, 0x01, 0x00, 0xFE, 0xFF, 0x00, 0x08, 0x7A, 0x0B, 0x31+1,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
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

    for (size_t idx = 0; idx < sizeof(BUFFER) - 1; idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
    ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[10]), DATA_TYPE_ANG_VEL);

    Hwt905_AngularVelocity_t actual_ang_vel;
    hwt905_driver.get_angular_velocity(&actual_ang_vel);
    ASSERT_NEAR(actual_ang_vel.wx, expected_ang_vel.wx, 0.001);
    ASSERT_NEAR(actual_ang_vel.wy, expected_ang_vel.wy, 0.001);
    ASSERT_NEAR(actual_ang_vel.wz, expected_ang_vel.wz, 0.001);
    ASSERT_NEAR(actual_ang_vel.temperature, expected_ang_vel.temperature, 0.01);
}

TEST(Hwt905Test, get_angular_velocity_wrong_checksum){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A, 0x0B, 0x2C+1,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
}

/**
 * @brief parse get_angle()
 */
TEST(Hwt905Test, get_angle_normal){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x53, 0xF4, 0xFF, 0xF5, 0xFF, 0xD3, 0xC0, 0x24, 0x29, 0x6F,
    };
    Hwt905_Angle_t expected_ang_vel = {
        .roll = 0.0000,
        .pitch = 0.0000,
        .yaw = -88.8,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER) - 1; idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
    ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[10]), DATA_TYPE_ANGLE);

    Hwt905_Angle_t actual_angle;
    hwt905_driver.get_angle(&actual_angle);

    ASSERT_NEAR(actual_angle.roll, expected_ang_vel.roll, 0.1);
    ASSERT_NEAR(actual_angle.pitch, expected_ang_vel.pitch, 0.1);
    ASSERT_NEAR(actual_angle.yaw, expected_ang_vel.yaw, 0.1);
}

TEST(Hwt905Test, get_angle_wrong_checksum){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x53, 0xF4, 0xFF, 0xF5, 0xFF, 0xD3, 0xC0, 0x24, 0x29, 0x6F+1,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
}

/**
 * @brief parse get_magnetic_field()
 */
TEST(Hwt905Test, get_magnetic_field_normal){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x54, 0x86, 0xF4, 0x37, 0x00, 0x92, 0xF8, 0x7A, 0x0B, 0x69,
    };
    Hwt905_Magnetic_t expected_mag = {
        .mag_x = 62598,
        .mag_y = 55,
        .mag_z = 63634,
        .temperature = 29.38,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER) - 1; idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
    ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[10]), DATA_TYPE_MAG);

    Hwt905_Magnetic_t actual_mag;
    hwt905_driver.get_magnetic_field(&actual_mag);

    ASSERT_EQ(actual_mag.mag_x, expected_mag.mag_x);
    ASSERT_EQ(actual_mag.mag_y, expected_mag.mag_y);
    ASSERT_EQ(actual_mag.mag_z, expected_mag.mag_z);
    ASSERT_NEAR(actual_mag.temperature, expected_mag.temperature, 0.1);
}

TEST(Hwt905Test, get_magnetic_field_wrong_checksum){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x54, 0x86, 0xF4, 0x37, 0x00, 0x92, 0xF8, 0x7A, 0x0B, 0x69 + 1,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
}

/**
 * @brief parse get_quaternion()
 */
TEST(Hwt905Test, get_quaternion_normal){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x59, 0x94, 0x49, 0xCF, 0x00, 0x7F, 0x00, 0xBA, 0x68, 0xFB,
    };
    Hwt905_Quaternion_t expected_quaternion = {
        .q_0 = 0.5748,
        .q_1 = 0.0063,
        .q_2 = 0.0039,
        .q_3 = 0.8182,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER) - 1; idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
    ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[10]), DATA_TYPE_QUATERNION);

    Hwt905_Quaternion_t actual_quaternion;
    hwt905_driver.get_quaternion(&actual_quaternion);

    ASSERT_NEAR(actual_quaternion.q_0, expected_quaternion.q_0, 0.001);
    ASSERT_NEAR(actual_quaternion.q_1, expected_quaternion.q_1, 0.001);
    ASSERT_NEAR(actual_quaternion.q_2, expected_quaternion.q_2, 0.001);
    ASSERT_NEAR(actual_quaternion.q_3, expected_quaternion.q_3, 0.001);
}

TEST(Hwt905Test, get_quaternion_wrong_checksum){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x59, 0x94, 0x49, 0xCF, 0x00, 0x7F, 0x00, 0xBA, 0x68, 0xFB + 1,
    };

    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        ASSERT_EQ(hwt905_driver.process_next_byte(BUFFER[idx]), DATA_TYPE_NONE);
    }
}

TEST(Hwt905Test, get_quaternion_roll_rotation_head_down){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x59, 0xAC, 0x68, 0x89, 0xDD, 0x3E, 0x14, 0x20, 0xC2, 0x5C,
        0x55, 0x59, 0x1D, 0x68, 0xDE, 0xDB, 0x5B, 0x15, 0x82, 0xC2, 0xA0,
        0x55, 0x59, 0xED, 0x67, 0xAA, 0xDA, 0xEB, 0x15, 0x1C, 0xC3, 0x65,
    };
    std::vector<Hwt905_Quaternion_t> expected_quaternions = {
        {.q_0 = 0.817749, .q_1 = -0.269257, .q_2 = 0.158142, .q_3 = -0.483398},
        {.q_0 = 0.813385, .q_1 = -0.282288, .q_2 = 0.166840, .q_3 = -0.480408},
        {.q_0 = 0.811920, .q_1 = -0.291687, .q_2 = 0.171234, .q_3 = -0.475708},
    };

    std::vector<Hwt905_Quaternion_t> actual_quaternions;
    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        if (DATA_TYPE_QUATERNION == hwt905_driver.process_next_byte(BUFFER[idx])) {
            Hwt905_Quaternion_t parsed_quaternion;
            hwt905_driver.get_quaternion(&parsed_quaternion);
            actual_quaternions.push_back(parsed_quaternion);
        }
    }

    for (size_t q_idx = 0; q_idx < 3; q_idx++) {
        ASSERT_NEAR(actual_quaternions[q_idx].q_0, expected_quaternions[q_idx].q_0, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_1, expected_quaternions[q_idx].q_1, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_2, expected_quaternions[q_idx].q_2, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_3, expected_quaternions[q_idx].q_3, 0.001);
    }
}

TEST(Hwt905Test, get_quaternion_roll_rotation_head_up){
    Hwt905Driver hwt905_driver;
    const uint8_t BUFFER[] = {
        0x55, 0x59, 0x7F, 0x63, 0xB2, 0x24, 0x21, 0xE7, 0xCA, 0xBC, 0xF4,
        0x55, 0x59, 0x12, 0x64, 0x63, 0x23, 0xF2, 0xE7, 0xA3, 0xBC, 0xE2,
        0x55, 0x59, 0xAC, 0x64, 0x1F, 0x21, 0xB2, 0xE8, 0x20, 0xBC, 0x74,
    };
    std::vector<Hwt905_Quaternion_t> expected_quaternions = {
        {.q_0 = 0.777313, .q_1 = 0.286682, .q_2 = -0.194305, .q_3 = -0.525085},
        {.q_0 = 0.781799, .q_1 = 0.276459, .q_2 = -0.187927, .q_3 = -0.526276},
        {.q_0 = 0.786499, .q_1 = 0.258759, .q_2 = -0.182068, .q_3 = -0.530273},
    };

    std::vector<Hwt905_Quaternion_t> actual_quaternions;
    for (size_t idx = 0; idx < sizeof(BUFFER); idx++) {
        if (DATA_TYPE_QUATERNION == hwt905_driver.process_next_byte(BUFFER[idx])) {
            Hwt905_Quaternion_t parsed_quaternion;
            hwt905_driver.get_quaternion(&parsed_quaternion);
            actual_quaternions.push_back(parsed_quaternion);
        }
    }

    for (size_t q_idx = 0; q_idx < 3; q_idx++) {
        ASSERT_NEAR(actual_quaternions[q_idx].q_0, expected_quaternions[q_idx].q_0, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_1, expected_quaternions[q_idx].q_1, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_2, expected_quaternions[q_idx].q_2, 0.001);
        ASSERT_NEAR(actual_quaternions[q_idx].q_3, expected_quaternions[q_idx].q_3, 0.001);
    }
}

int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
