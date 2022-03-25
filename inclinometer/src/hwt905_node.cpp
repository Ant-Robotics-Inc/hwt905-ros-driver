/**
 * @file hwt905_node.cpp
 * @author ponomarevda96@gmail.com
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


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
    InclinometerDriverRos inclinometer_driver_ros(nh);

    ros::Rate loop_rate(1);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
        inclinometer_driver_ros.publish();
    }

    return 0;
}
