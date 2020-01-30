#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hrwros_msgs/BoxHeightInformation.h"
#include "hrwros_msgs/ConvertMetresToFeet.h"

#include <functional>

void box_height_info_cb(const boost::shared_ptr<hrwros_msgs::BoxHeightInformation const> &data) {
    ros::NodeHandle ros_node;
    auto metres_to_feet = ros_node.serviceClient<hrwros_msgs::ConvertMetresToFeet>("metres_to_feet");
    hrwros_msgs::ConvertMetresToFeet srv;
    srv.request.distance_metres = data->box_height;
    bool result = metres_to_feet.call(srv);
    if (result) {
        auto conversion_result = srv.response.distance_feet;
        ROS_INFO("Distance %4.2f feet", conversion_result);
    } else
        ROS_ERROR("Service call failed fro service metres_to_feet");

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "box_height_in_feet");
    ros::NodeHandle ros_node;

    ROS_INFO("Waiting for service... (C++)");
    bool available = ros::service::waitForService("metres_to_feet");
    if (!available) {
        ROS_ERROR("Service metres_to_feet not available.");
        exit(1);
    }
    ROS_INFO("Service %s is now available!", "metres_to_feet");

    ros::Subscriber sub = ros_node.subscribe<hrwros_msgs::BoxHeightInformation>("/box_height_info", 10, box_height_info_cb);

    ros::spin();

    return 0;
}
