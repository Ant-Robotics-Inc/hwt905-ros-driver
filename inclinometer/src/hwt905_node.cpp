/**
 * @file hwt905_node.cpp
 * @author ponomarevda96@gmail.com
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <hwt905_driver.hpp>
#include <serial_driver.hpp>


class InclinometerDriverRos {
public:
    InclinometerDriverRos(ros::NodeHandle& ros_node);
    void publish();
private:
    ros::NodeHandle& _ros_node;
    ros::Publisher _ros_pub;
    sensor_msgs::Imu _msg;
};


InclinometerDriverRos::InclinometerDriverRos(ros::NodeHandle& ros_node): _ros_node(ros_node) {
    _ros_pub = ros_node.advertise<sensor_msgs::Imu>("/inclinometer", 5);

    _msg.header.stamp = ros::Time::now();
    _msg.header.frame_id = "world";
    _msg.orientation.x = 0.0;
    _msg.orientation.y = 0.0;
    _msg.orientation.z = 0.0;
    _msg.orientation.w = 1.0;
}

void InclinometerDriverRos::publish() {
    _msg.header.stamp = ros::Time::now();
    _msg.header.frame_id = "world";

    _ros_pub.publish(_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hwt905_node");
    ros::NodeHandle nh;
    InclinometerDriverRos ros_driver(nh);
    Hwt905Driver hwt905_driver;
    SerialDriver serial_driver("/dev/ttyUSB0", 1000000);

    constexpr const size_t MAX_SERIAL_BUFFER_RECV_SIZE = 256;
    uint8_t serial_recv_buf[MAX_SERIAL_BUFFER_RECV_SIZE];

    size_t num_of_recv_bytes;
    ros::Rate loop_rate(200);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
        ros_driver.publish();
        num_of_recv_bytes = serial_driver.spin(serial_recv_buf, MAX_SERIAL_BUFFER_RECV_SIZE);

        for (size_t byte_idx = 0; byte_idx < num_of_recv_bytes; byte_idx++) {
            hwt905_driver.process(serial_recv_buf[byte_idx]);
        }
    }

    return 0;
}
