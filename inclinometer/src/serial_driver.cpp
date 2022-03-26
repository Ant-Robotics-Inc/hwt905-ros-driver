/**
 * @file serial_driver.cpp
 * @author ponomarevda96@gmail.com
 */

#include <serial_driver.hpp>

#include <ros/ros.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>


void SerialDriver::init(const std::string& port, uint32_t baudrate) {
    // open
    _fd_serial_port = open(port.c_str(), O_RDWR);
    if (_fd_serial_port < 0) {
        ROS_ERROR_STREAM("Error " << errno << " from open: " << strerror(errno));
    } else {
        ROS_INFO_STREAM("Serial port " << port << " has been successfully open.");
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
    
    uint32_t serial_speed;
    switch (baudrate) {
        case 9600:
            serial_speed = B9600;
            break;
        case 19200:
            serial_speed = B19200;
            break;
        case 38400:
            serial_speed = B38400;
            break;
        case 57600:
            serial_speed = B57600;
            break;
        case 115200:
            serial_speed = B115200;
            break;
        case 230400:
            serial_speed = B230400;
            break;
        case 460800:
            serial_speed = B460800;
            break;
        case 921600:
            serial_speed = B921600;
            ROS_ERROR_STREAM("Serial port speed is ok");
            break;
        case 1000000:
            serial_speed = B1000000;
            break;
        default:
            serial_speed = 0;
            break;
    }
    if (serial_speed) {
        ROS_INFO_STREAM("Serial port: selected baudrate is " << baudrate);
    } else {
        serial_speed = B9600;
        ROS_ERROR_STREAM("Serial port: unsopported baudrate " << baudrate << ". Use default 9600");
    }
    cfsetispeed(&tty, serial_speed);
    cfsetospeed(&tty, serial_speed);

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
    int n = read(_fd_serial_port, recv_buf, max_buf_size);
    if (n < 0) {
        ROS_ERROR_STREAM_THROTTLE(1, "Recv error: " << n);
    }

    return n;
}
