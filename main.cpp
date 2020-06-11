#include <lipm_motion/lipm.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_motion_node");
    ros::NodeHandle nh;
 
    RobotParameters robot;
    lipm* lm;
    
    lm = new lipm(nh,robot);

    static ros::Rate rate(50);
    while (ros::ok())
    {
        lm->publishPath();
        rate.sleep();
    }
    delete lm;
    return 0;
}