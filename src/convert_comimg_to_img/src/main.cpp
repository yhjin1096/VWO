#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedImageConverter {
public:
    CompressedImageConverter()
        : nh_(),
          image_pub_(nh_.advertise<sensor_msgs::Image>("/VPS/image_color", 1)),
          compressed_image_sub_(nh_.subscribe("/VPS/image_color/compressed", 1, &CompressedImageConverter::compressedImageCallback, this)) {}

private:
    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_msg) {
        try {
            // 압축 이미지를 OpenCV의 cv::Mat로 디코딩
            cv::Mat decoded_image = cv::imdecode(cv::Mat(compressed_msg->data), cv::IMREAD_COLOR);

            if (decoded_image.empty()) {
                ROS_WARN("Decoded image is empty.");
                return;
            }

            // cv_bridge를 사용하여 sensor_msgs::Image로 변환
            std_msgs::Header header = compressed_msg->header; // 기존 헤더 유지
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", decoded_image).toImageMsg();

            // 변환된 이미지 퍼블리시
            image_pub_.publish(image_msg);
        } catch (const cv::Exception& e) {
            ROS_ERROR("cv::Exception: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("std::exception: %s", e.what());
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    ros::Subscriber compressed_image_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "compressed_image_to_image_converter");
    CompressedImageConverter converter;
    ros::spin();
    return 0;
}