#include "fbrtkros/fbrtk_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fbrtk_driver_node");
    ros::NodeHandle nh("~");
    
    fbrtkros::FBRTKDriver driver(nh);
    
    if (!driver.initialize()) {
        ROS_ERROR("Failed to initialize FB-RTK driver");
        return -1;
    }
    
    ROS_INFO("FB-RTK driver started");
    driver.spin();
    
    return 0;
}
