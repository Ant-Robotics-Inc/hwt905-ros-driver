/**
 * @file serial_driver.cpp
 * @author ponomarevda96@gmail.com
 */

#include <serial_driver.hpp>

#include <ros/ros.h>

#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <sys/file.h>


SerialDriver::SerialDriver(const std::string& port, uint32_t baudrate) :
        _port(port), _baudrate(baudrate) {

    // open
    _fd_serial_port = open(port.c_str(), O_RDWR);
    if (_fd_serial_port < 0) {
        ROS_ERROR_STREAM("Error " << errno << " from open: " << strerror(errno));
    } else {
        ROS_ERROR_STREAM("Serial port " << port << " has been successfully open.");
    }

    // read settings
    struct termios tty;
    if(tcgetattr(_fd_serial_port, &tty) != 0) {
        ROS_ERROR_STREAM("Error " << errno << " from tcgetattr: " << strerror(errno));
    }

    // common configuration
    tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size 
    tty.c_cflag |= CS8;             // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;           // Disable echo
    tty.c_lflag &= ~ECHOE;          // Disable erasure
    tty.c_lflag &= ~ECHONL;         // Disable new-line echo
    tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;          // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;          // Prevent conversion of newline to carriage return/line feed

    // configurate timeouts
    tty.c_cc[VTIME] = 0;            // No blocking
    tty.c_cc[VMIN] = 0;
    
    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);

    if (tcsetattr(_fd_serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
    }

    if (flock(_fd_serial_port, LOCK_EX | LOCK_NB) == -1) {
        ROS_ERROR_STREAM("Serial port with file descriptor " << std::to_string(_fd_serial_port) << " is already locked by another process.");
    }
}

SerialDriver::~SerialDriver() {
    close(_fd_serial_port);
}

int SerialDriver::spin(uint8_t recv_buf[], size_t max_buf_size) {
    int n = read(_fd_serial_port, &recv_buf, sizeof(max_buf_size));
    if (n < 0) {
        ROS_ERROR_STREAM_THROTTLE(1, "Recv error: " << n);
    }

    return n;
}

int get_recv_data(uint8_t dest[]) {
    
}