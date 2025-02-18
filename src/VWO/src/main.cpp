#include <VWO/main.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "VWO_node");
    VWO_node vwo_node;

    ros::spin();

    return 0;
}