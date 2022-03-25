/**
 * @file hwt905_driver.hpp
 * @author Dmitry Ponomarev
 * @brief Based on https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Datasheet.pdf
 */

#ifndef HW905_DRIVER_HPP
#define HW905_DRIVER_HPP

#include <stdint.h>
#include <cstddef>


#define START_BYTE              0x55
#define START_BYTE_TIME         0x50
#define START_BYTE_ACCEL        0x51
#define START_BYTE_ANG_VEL      0x52
#define START_BYTE_ANGLE        0x53
#define START_BYTE_MAG          0x54
#define START_BYTE_QUATERNION   0x55

// enum DataTypeHeader_t : uint16_t {
//     HEADER_TIME         = (START_BYTE << 8) | START_BYTE_TIME,
//     HEADER_ACCEL        = (START_BYTE << 8) | START_BYTE_ACCEL,
//     HEADER_ANG_VEL      = (START_BYTE << 8) | START_BYTE_ANG_VEL,
//     HEADER_ANGLE        = (START_BYTE << 8) | START_BYTE_ANGLE,
//     HEADER_MAG          = (START_BYTE << 8) | START_BYTE_MAG,
//     HEADER_QUATERNION   = (START_BYTE << 8) | START_BYTE_QUATERNION,
// };

enum DataTypeHeader_t : uint16_t {
    HEADER_TIME         = (START_BYTE) | (START_BYTE_TIME << 8),
    HEADER_ACCEL        = (START_BYTE) | (START_BYTE_ACCEL << 8),
    HEADER_ANG_VEL      = (START_BYTE) | (START_BYTE_ANG_VEL << 8),
    HEADER_ANGLE        = (START_BYTE) | (START_BYTE_ANGLE << 8),
    HEADER_MAG          = (START_BYTE) | (START_BYTE_MAG << 8),
    HEADER_QUATERNION   = (START_BYTE) | (START_BYTE_QUATERNION << 8),
};


enum DataType_t {
    DATA_TYPE_NONE,
    DATA_TYPE_TIME,
    DATA_TYPE_ACCEL,
    DATA_TYPE_ANG_VEL,
    DATA_TYPE_ANGLE,
    DATA_TYPE_MAG,
    DATA_TYPE_QUATERNION,
    NUMBER_OF_DATA_TYPES,
};

class Hwt905Driver;

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
    uint8_t msL;
    uint8_t msH;
    uint8_t SUM;
};


///< 5.1.2 Acceleration Output
struct Acceleration_t {
    uint16_t header;
    uint8_t AxL;
    uint8_t AxH;
    uint8_t AyL;
    uint8_t AyH;
    uint8_t AzL;
    uint8_t AzH;
    uint8_t TL;
    uint8_t TH;
    uint8_t SUM;
};

///< 5.1.3 Angular Velocity Output
struct AngularVelocity_t {
    uint16_t header;
    uint8_t wxL;
    uint8_t wxH;
    uint8_t wyL;
    uint8_t wyH;
    uint8_t wzL;
    uint8_t wzH;
    uint8_t TL;
    uint8_t TH;
    uint8_t sum;
};

///< 5.1.4  Angle Output
struct Angle_t {
    uint16_t header;
    uint8_t RollL;
    uint8_t RollH;
    uint8_t PitchL;
    uint8_t PitchH;
    uint8_t YawL;
    uint8_t YawH;
    uint8_t VL;
    uint8_t VH;
    uint8_t sum;
};

///< 5.1.5 Magnetic Output
struct Magnetic_t {
    uint16_t header;
    uint8_t HxL;
    uint8_t HxH;
    uint8_t HyL;
    uint8_t HyH;
    uint8_t HzL;
    uint8_t HzH;
    uint8_t TL;
    uint8_t TH;
    uint8_t sum;
};

///< 5.1.6 Quaternion Output
struct Quaternion_t {
    uint16_t header;
    uint8_t Q0L;
    uint8_t Q0H;
    uint8_t Q1L;
    uint8_t Q1H;
    uint8_t Q2L;
    uint8_t Q2H;
    uint8_t Q3L;
    uint8_t Q3H;
    uint8_t sum;
};
#pragma pack(pop)


class Hwt905Driver {
public:
    Hwt905Driver() {};
    DataType_t process(uint8_t byte);

    static constexpr const size_t PAYLOAD_SIZE = 11;
    static void linearize_ring_buffer(uint8_t linear_buf[PAYLOAD_SIZE],
                                      const uint8_t ring_buf[PAYLOAD_SIZE],
                                      size_t ring_buf_idx);
private:
    void _linearize_ring_buffer();
    DataType_t check_payload();

    uint8_t _ring_buffer[PAYLOAD_SIZE];
    uint8_t _linear_buffer[PAYLOAD_SIZE];
    size_t _ring_buffer_idx = 0;
};

#endif  // HW905_DRIVER_HPP