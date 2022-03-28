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
    InclinometerDriverRos(ros::NodeHandle& ros_node, Hwt905Driver& hwt905_driver);
    void publish();
    void process_parsed_result(Hwt905_DataType_t data_type);
private:
    ros::NodeHandle& _ros_node;
    Hwt905Driver& _hwt905_driver;
    ros::Publisher _ros_pub;
    sensor_msgs::Imu _imu_msg;

    Hwt905_Time_t _time;
    Hwt905_Acceleration_t _accel;
    Hwt905_AngularVelocity_t _ang_vel;
    Hwt905_Angle_t _angle;
    Hwt905_Magnetic_t _mag;
    Hwt905_Quaternion_t _quaternion;
};


InclinometerDriverRos::InclinometerDriverRos(ros::NodeHandle& ros_node, Hwt905Driver& hwt905_driver):
        _ros_node(ros_node), _hwt905_driver(hwt905_driver) {
    _ros_pub = ros_node.advertise<sensor_msgs::Imu>("/imu", 5);

    _imu_msg.header.stamp = ros::Time::now();
    _imu_msg.header.frame_id = "base_link";

    // horizontal
    _quaternion.q_0 = 0;
    _quaternion.q_1 = 0;
    _quaternion.q_2 = 0;
    _quaternion.q_3 = 1.0;
}

void InclinometerDriverRos::publish() {
    _imu_msg.header.stamp = ros::Time::now();
    _imu_msg.header.frame_id = "base_link";

    _imu_msg.orientation.x = _quaternion.q_1;
    _imu_msg.orientation.y = _quaternion.q_2;
    _imu_msg.orientation.z = _quaternion.q_3;
    _imu_msg.orientation.w = _quaternion.q_0;

    _imu_msg.angular_velocity.x = _ang_vel.wx * 3.14 / 180;
    _imu_msg.angular_velocity.y = _ang_vel.wy * 3.14 / 180;
    _imu_msg.angular_velocity.z = _ang_vel.wz * 3.14 / 180;

    _imu_msg.linear_acceleration.x = _accel.ax;
    _imu_msg.linear_acceleration.y = _accel.ay;
    _imu_msg.linear_acceleration.z = _accel.az;

    _ros_pub.publish(_imu_msg);
}

void InclinometerDriverRos::process_parsed_result(Hwt905_DataType_t data_type) {
    switch (data_type) {
        case DATA_TYPE_NONE:
            ///< Although here we do nothing, typically it is the most common case. Keep it first.
            break;
        case DATA_TYPE_TIME:
            _hwt905_driver.get_time(&_time);
            break;
        case DATA_TYPE_ACCEL:
            _hwt905_driver.get_acceleration(&_accel);
            break;
        case DATA_TYPE_ANG_VEL:
            _hwt905_driver.get_angular_velocity(&_ang_vel);
            break;
        case DATA_TYPE_ANGLE:
            _hwt905_driver.get_angle(&_angle);
            break;
        case DATA_TYPE_MAG:
            _hwt905_driver.get_magnetic_field(&_mag);
            break;
        case DATA_TYPE_QUATERNION:
            _hwt905_driver.get_quaternion(&_quaternion);
            break;
        default:
            break;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hwt905_node");
    ros::NodeHandle nh;
    Hwt905Driver hwt905_driver;
    SerialDriver serial_driver;

    std::string serial_port;
    int32_t baudrate;
    if (!nh.getParam("port", serial_port) || !nh.getParam("baudrate", baudrate)) {
        ROS_ERROR_STREAM("Inclinometer. 'port' or 'baudrate' parameter is not specified. Abort.");
        return 0;
    }
    serial_driver.init(serial_port, baudrate);

    InclinometerDriverRos ros_driver(nh, hwt905_driver);

    constexpr const size_t MAX_SERIAL_BUFFER_RECV_SIZE = 256;
    uint8_t serial_recv_buf[MAX_SERIAL_BUFFER_RECV_SIZE];

    size_t num_of_recv_bytes_amount = 0;
    int32_t num_of_recv_bytes;
    ros::Rate loop_rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        ros_driver.publish();

        num_of_recv_bytes = serial_driver.spin(serial_recv_buf, MAX_SERIAL_BUFFER_RECV_SIZE);
        if (num_of_recv_bytes < 0) {
            continue;   // handle error
        }

        num_of_recv_bytes_amount += num_of_recv_bytes;

        for (size_t byte_idx = 0; byte_idx < num_of_recv_bytes; byte_idx++) {
            auto data_type = hwt905_driver.process_next_byte(serial_recv_buf[byte_idx]);
            ros_driver.process_parsed_result(data_type);
        }
    }

    return 0;
}
