#include <VWO/VWO_node.hpp>

VWO_node::VWO_node()
{
    getConfig();
    
    image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/VPS/image_color", 1);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *odom_sub);
    sync->registerCallback(boost::bind(&VWO_node::syncCallback, this, _1, _2));
}

void VWO_node::getConfig()
{
    std::string config_file;
    if (!nh_.getParam("config_file", config_file))
    {
        ROS_ERROR("Failed to get 'config_file' from parameter server.");
        return;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        // 카메라 매개변수 읽기
        YAML::Node camera = config["Camera"];
        double fx = camera["fx"].as<double>();
        double fy = camera["fy"].as<double>();
        double cx = camera["cx"].as<double>();
        double cy = camera["cy"].as<double>();

        ROS_INFO("Camera fx: %f, fy: %f, cx: %f, cy: %f", fx, fy, cx, cy);

        // ORB 매개변수 읽기
        YAML::Node feature = config["Feature"];
        int num_levels = feature["num_levels"].as<int>();
        ROS_INFO("ORB num_levels: %d", num_levels);
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to load or parse YAML file: %s", e.what());
    }
}

void VWO_node::syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::imshow("grapy", gray_image);
    char k = cv::waitKey(1);
    if(k == 'q')
    {
        ros::shutdown();
        exit(0);
    }
}