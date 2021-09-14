#include <lipm_motion/lipm_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_motion_node");
    ros::NodeHandle nh;
    lipm_ros* lm;
    lm = new lipm_ros(nh);
    ros::spin();
    delete lm;
    return 0;
}