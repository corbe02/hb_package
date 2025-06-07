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
    ParticleFilter(ros::NodeHandle &nh, int numParticles, double initMean, double initStd);
    void imageCallback(const std_msgs::Float32::ConstPtr &msg);

    
private:

    ros::Publisher pub;
    ros::NodeHandle private_nh_;         
    ros::Subscriber image_sub_; 
    std::ofstream log_file;

    std::vector<Particle> particles;
    int N;
    double processNoise;
    double measNoise;
    double meanInit, stdInit;
    double measurementNoise;

    void init();
    void normalizeWeights();
    void resample();

    void predict(double processNoise);
    void updateWeights(double measurement, double measNoiseStd);
    double estimate() const;
};

#endif // PARTICLE_FILTER_H