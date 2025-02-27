#include <VWO/VWO_node.h>

Eigen::Matrix4d odometryToEigen(const nav_msgs::OdometryConstPtr& odom) {
    Mat44_t T = Eigen::Matrix4d::Identity();

    // translation: 위치 정보를 추출합니다.
    T(0,3) = odom->pose.pose.position.x;
    T(1,3) = odom->pose.pose.position.y;
    T(2,3) = odom->pose.pose.position.z;

    // rotation: 쿼터니언을 이용해 회전 행렬로 변환합니다.
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    
    T.block<3,3>(0,0) = R;

    return T;
}

VWO_node::VWO_node()
{
    // launch 파일에서 
    ros::NodeHandle tmp_nh("~");
    tmp_nh.getParam("config_file_path", config_file_path_);
    tmp_nh.getParam("vocabulary_path", vocabulary_path_);
    tmp_nh.getParam("image_topic", image_topic_name_);
    tmp_nh.getParam("odom_frame", odom_name_);
    tmp_nh.getParam("base_link_frame", base_link_frame_);
    tmp_nh.getParam("camera_link_frame", camera_link_frame);
    
    config_ = std::make_shared<Config>(config_file_path_);
    system_ = std::make_shared<System>(config_, vocabulary_path_);
    // system_->run(); // map 관리, visualizer, optimization 등

    tf_ = std::make_unique<tf2_ros::Buffer>(); 
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // sub_ = nh_.subscribe(image_topic_name_, 1, &VWO_node::imageCallback, this);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 1);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_name_, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *odom_sub_);
    sync->registerCallback(boost::bind(&VWO_node::syncCallback2, this, _1, _2));
}

// pose는 curr -> prev
void VWO_node::publishPose(Mat44_t pose, const ros::Time& stamp)
{
    auto base_link_to_camera_ = tf_->lookupTransform(base_link_frame_, camera_link_frame, stamp, ros::Duration(10.0));
    base_link_to_camera_affine_ = tf2::transformToEigen(base_link_to_camera_.transform);
    
    //prev_odom curr_odom 차이 -> scale
    //pose의 scale
    Mat44_t relative_odom = curr_odom_.inverse() * prev_odom_;
    double odom_scale = relative_odom.block<3, 1>(0, 3).norm();
    double cam_scale = pose.block<3, 1>(0, 3).norm();
    double scale = odom_scale / cam_scale;
    
    pose(0, 3) *= scale;
    pose(1, 3) *= scale;
    pose(2, 3) *= scale;

    // odom_to_base * base_to_camera -> 이전 카메라 위치(tf 계산) * pose 변화량(camera 계산)
    Mat44_t pose_oc = prev_odom_ * base_link_to_camera_affine_.matrix() * pose.inverse();

    Eigen::Matrix3d R = pose_oc.block<3,3>(0,0);
    Eigen::Vector3d t = pose_oc.block<3,1>(0,3);

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
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "camera"));

    // prev odom  curr odom
}

void VWO_node::publishPose(Mat44_t pose_wc)
{
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
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "camera"));
}

void VWO_node::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    const double timestamp = image_msg->header.stamp.toSec();
    image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::resize(image_, vis_image_, image_.size()/2);
    cv::imshow("vis", vis_image_);
    char k = cv::waitKey(1);

    if(k == 'q')
        exit(0);

    std::shared_ptr<Mat44_t> pose_cw = system_->trackFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image, timestamp);
}

void VWO_node::syncCallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    const double timestamp = image_msg->header.stamp.toSec();
    // system_->trackFrame(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image, timestamp);
    
    image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::resize(image_, vis_image_, image_.size()/2);
    
    cv::imshow("vis", vis_image_);
    char k = cv::waitKey(1);

    if(k == 's')
    {
        prev_image_ = image_.clone();
        prev_timestamp_ = timestamp;
        prev_odom_ = odometryToEigen(odom_msg);
    }
    else if(k == 'q')
    {
        exit(0);
    }
    // else if (k == 'e')
    // {
        curr_image_ = image_.clone();
        curr_timestamp_ = timestamp;
        curr_odom_ = odometryToEigen(odom_msg);
    // }

    if(!prev_image_.empty() && !curr_image_.empty())
    {
        std::shared_ptr<Mat44_t> pose_cw = system_->trackTwoFrame(prev_image_, curr_image_, prev_timestamp_, curr_timestamp_);
        publishPose(*pose_cw, image_msg->header.stamp);
    }
}

void VWO_node::syncCallback2(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    const double timestamp = image_msg->header.stamp.toSec();
    image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::resize(image_, vis_image_, image_.size()/2);
    cv::imshow("vis", vis_image_);
    char k = cv::waitKey(1);

    if(k == 'q')
        exit(0);

    Eigen::Affine3d base_link_to_camera_affine;
    try
    {
        auto base_link_to_camera_ = tf_->lookupTransform(base_link_frame_, camera_link_frame, image_msg->header.stamp, ros::Duration(10.0));
        base_link_to_camera_affine = tf2::transformToEigen(base_link_to_camera_.transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("Transform failed: %s", ex.what());
    }

    if(base_link_to_camera_affine.matrix().cols() != 0 && base_link_to_camera_affine.matrix().rows() != 0)
    {
        curr_odom_ = odometryToEigen(odom_msg); // odom to base
        Mat44_t curr_cam_tf = curr_odom_ * base_link_to_camera_affine.matrix();
        std::shared_ptr<Mat44_t> pose_wc = system_->trackFrameWithOdom(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image, timestamp, curr_cam_tf);
        std::cout << *pose_wc << std::endl;
        publishPose(*pose_wc);
    }
}