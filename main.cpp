#include <lipm_motion/lipm.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_motion_node");
    ros::NodeHandle nh;
    lipm* lm;
    lm = new lipm(nh);
    ros::spin();
    delete lm;
    return 0;
}