#include <VWO/VWO_node.hpp>

VWO_node::VWO_node()
{
    // launch 파일에서 
    ros::NodeHandle tmp_nh("~");
    tmp_nh.getParam("config_file_path", config_file_path_);
    tmp_nh.getParam("image_topic", image_topic_name_);
    tmp_nh.getParam("odom_frame", odom_name_);
    
    config_ = std::make_shared<Config>(config_file_path_);
    system_ = std::make_shared<System>(config_);

    image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 1);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_name_, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *odom_sub);
    sync->registerCallback(boost::bind(&VWO_node::syncCallback, this, _1, _2));
}

void VWO_node::syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    
    cv::imshow("gray", gray_image);
    char k = cv::waitKey(1);
    if(k == 'q')
    {
        ros::shutdown();
        exit(0);
    }
    else if (k == 's')
    {
        cv::Mat descriptor;
        cv::Mat mask = cv::Mat();
        std::vector<cv::KeyPoint> keypts;
        
        system_->feature_extractor_->extractFeatures(gray_image, mask, keypts, descriptor);
        std::cout << keypts.size() << std::endl;
        cv::imshow("desc", descriptor);

        // extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    }
}