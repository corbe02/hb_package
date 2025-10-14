#include "ParticleFilter.hpp"  
#include "PFSameni.hpp"
#include "PF_new_model.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle nh;

    //PFSameni pf(nh, 500);
    //ParticleFilter pf(nh, 200);
    Particle_new pn(nh, 1000);


    ros::spin();  
    return 0;
}
