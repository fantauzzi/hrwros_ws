#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hrwros_msgs/BoxHeightInformation.h"
#include "hrwros_msgs/SensorInformation.h"

#include <functional>

/**
 * Adapted from the ROS tutorial demonstrating simple sending of messages over the ROS system.
 *
 * This ROS node will receive measurements from topic /sensor_info, compute the corresponding box
 * height and, if the measurement is outside the error range, publish the computed box height on topic /box_height_info.
 */

void sensor_info_callback(const boost::shared_ptr<hrwros_msgs::SensorInformation const> &msg,
                          ros::Publisher &bhi_publisher) {
    auto height_box = 2. - msg->sensor_data.range;
    if (height_box >= 1) {
        auto box_height_info = hrwros_msgs::BoxHeightInformation();
        box_height_info.box_height = height_box;
        bhi_publisher.publish(box_height_info);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compute_box_height");
    ros::NodeHandle ros_node;

    ROS_INFO("Waiting for topic %s to be published... (cpp)", "/sensor_info");


    ros::topic::waitForMessage<hrwros_msgs::SensorInformation>("/sensor_info");

    ROS_INFO("%s topic is now available!", "/sensor_info");

    ros::Publisher bhi_publisher = ros_node.advertise<hrwros_msgs::BoxHeightInformation>("/box_height_info", 10);

    /* Make a callback that calls sensor_info_callback() in turn, but hard-wires the second argument passed to it,
     * setting it to  bhi_publisher. Another option would be to use std::bind(). */
    auto callback = [&bhi_publisher](const boost::shared_ptr<hrwros_msgs::SensorInformation const> &msg) {
        sensor_info_callback(msg, bhi_publisher);
    };

    ros::Subscriber sub = ros_node.subscribe<hrwros_msgs::SensorInformation>("sensor_info", 10, callback);

    ros::spin();

    return 0;
}