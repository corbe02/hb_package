#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include "particle.hpp"
#include "utils.hpp"
#include <fstream>



class ParticleFilter{
public:
    ParticleFilter(ros::NodeHandle &nh, int numParticles);
    void imageCallback(const std_msgs::Float32::ConstPtr &msg);
    std::vector<Particle> particles;

    
private:

    ros::Publisher pub_;
    ros::NodeHandle private_nh_;         
    ros::Subscriber image_sub_; 
    std::ofstream log_file_;

    
    int N_;
    double process_noise_;
    double measurement_noise_;
    double sampling_rate_;


    void init();
    void normalizeWeights();
    void resample();

    void predict();
    void updateWeights(double measurement, double measNoiseStd);
    double estimate() const;
};

#endif // PARTICLE_FILTER_H