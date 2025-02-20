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
    // system_->run(); // map 관리, visualizer, optimization 등

    sub = nh_.subscribe(image_topic_name_, 1, &VWO_node::imageCallback, this);

    // image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 1);
    // odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_name_, 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *odom_sub);
    // sync->registerCallback(boost::bind(&VWO_node::syncCallback, this, _1, _2));
}
void VWO_node::publishPose(const Mat44_t& pose, const ros::Time& stamp)
{
    Mat44_t pose_wc = pose.inverse();
    Eigen::Matrix3d R = pose_wc.block<3,3>(0,0);
    Eigen::Vector3d t = pose_wc.block<3,1>(0,3);

    // Eigen 회전 행렬을 tf::Quaternion으로 변환
    tf::Matrix3x3 tf3d(
        R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2)
    );
    tf::Quaternion q;
    tf3d.getRotation(q);

    // tf::Transform에 평행 이동 및 회전 설정
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    transform.setRotation(q);

    // 현재 시간, 부모 프레임과 자식 프레임 설정하여 변환 publish
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));
}

void VWO_node::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    const double timestamp = image_msg->header.stamp.toSec();
    // system_->trackFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image, timestamp);
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat vis;
    cv::resize(image, vis, image.size()/2);
    cv::imshow("vis", vis);
    char k = cv::waitKey(1);
    if(k == 's')
    {
        prev_image = image.clone();
        prev_timestamp = timestamp;
    }
    else if(k == 'q')
    {
        exit(0);
    }
    // else if (k == 'e')
    // {
        curr_image = image.clone();
        curr_timestamp = timestamp;
    // }

    if(!prev_image.empty() && !curr_image.empty())
    {
        std::shared_ptr<Mat44_t> pose_cw = system_->trackTwoFrame(prev_image, curr_image, prev_timestamp, curr_timestamp);
        publishPose(*pose_cw, image_msg->header.stamp);
        // if(succeed)
        // {
        //     prev_image.release();
        //     curr_image.release();
        // }
        // else
        // {
        //     std::cout << "failed tracking" << std::endl;
        // }
    }
}

void VWO_node::syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    
    {
        cv::Mat descriptor;    
        cv::Mat mask = cv::Mat();
        std::vector<cv::KeyPoint> keypts;
        
        system_->feature_extractor_->extractFeatures(gray_image, mask, keypts, descriptor);
        cv::Mat tmp_image;
        cv::drawKeypoints(gray_image, keypts, tmp_image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
        cv::resize(tmp_image, tmp_image, tmp_image.size()/2);
        cv::imshow("tmp_image",tmp_image);
        // cv::imshow("desc", descriptor);
    }

    char k = cv::waitKey(1);
    if(k == 'q')
    {
        ros::shutdown();
        exit(0);
    }
}