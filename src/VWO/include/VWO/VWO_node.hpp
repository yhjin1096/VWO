#ifndef VWO_NODE_HPP
#define VWO_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "VWO/system.hpp"
#include "VWO/config.hpp"

class VWO_node
{
    public:
        VWO_node();
        std::shared_ptr<Config> config_;
        std::shared_ptr<System> system_;

        // ros param
        std::string config_file_path_;
        std::string image_topic_name_;
        std::string odom_name_;

    private:
        ros::NodeHandle nh_;
        message_filters::Subscriber<sensor_msgs::Image> *image_sub;
        message_filters::Subscriber<nav_msgs::Odometry> *odom_sub;
        void syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg);
};

#endif