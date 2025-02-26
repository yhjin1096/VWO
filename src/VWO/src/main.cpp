#include <VWO/main.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "VWO_node");
    VWO_node vwo_node;

    // ros::spin();
    ros::Rate rate(60);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}