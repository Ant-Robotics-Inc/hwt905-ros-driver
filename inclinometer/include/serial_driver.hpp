/**
 * @file serial_driver.hpp
 * @author Dmitry Ponomarev
 */

#ifndef SERIAL_DRIVER_HPP
#define SERIAL_DRIVER_HPP

#include <stdint.h>
#include <string>

class SerialDriver {
public:
    SerialDriver(const std::string& port, uint32_t baudrate);
    ~SerialDriver();
    int spin(uint8_t recv_buf[], size_t max_buf_size);
private:
    std::string _port;
    int _fd_serial_port;
    uint32_t _baudrate;
};

#endif  // SERIAL_DRIVER_HPP