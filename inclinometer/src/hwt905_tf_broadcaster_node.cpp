/**
 * @file hw905_tf_broadcaster.cpp
 * @author ponomarevda96@gmail.com
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>


void publish_state(float q_w, float q_x, float q_y, float q_z) {
    static tf2_ros::TransformBroadcaster tfPub_;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "base_link";
    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;
    transform.transform.rotation.x = q_x;
    transform.transform.rotation.y = q_y;
    transform.transform.rotation.z = q_z;
    transform.transform.rotation.w = q_w;
    transform.child_frame_id = "/imu";
    tfPub_.sendTransform(transform);
}


void imu_cb(const sensor_msgs::Imu::ConstPtr& imu) {
    float q_w = imu->orientation.w;
    float q_x = imu->orientation.x;
    float q_y = imu->orientation.y;
    float q_z = imu->orientation.z;

    publish_state(q_w, q_x, q_y, q_z);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "inclinometer_tf_broadcaster");
    ros::NodeHandle nh;
    auto imu_sub = nh.subscribe("/imu", 1, imu_cb);

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
