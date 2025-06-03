#include "ParticleFilter.hpp"  
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle nh;

    ParticleFilter pf(nh, 500, 0.0, 1.0);

    ros::spin();  // loop infinito gestito via callback
    return 0;
}
