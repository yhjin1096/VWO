#ifndef VWO_NODE_HPP
#define VWO_NODE_HPP

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        std::string base_link_frame_;
        std::string camera_link_frame;

    private:
        ros::NodeHandle nh_;

        // message callbacck - image, odometry
        cv::Mat image_, vis_image_;

        ros::Subscriber sub_;
        void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

        message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
        void syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg);

        cv::Mat prev_image_, curr_image_;
        double prev_timestamp_, curr_timestamp_;

        // publish pose result
        tf::TransformBroadcaster br_;
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        Eigen::Affine3d base_link_to_camera_affine_;
        Mat44_t prev_odom_, curr_odom_;
        void publishPose(Mat44_t pose, const ros::Time& stamp);
};

#endif