#include "ParticleFilter.hpp"  
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle nh;

    ParticleFilter pf(nh, 200);

    ros::spin();  
    return 0;
}
