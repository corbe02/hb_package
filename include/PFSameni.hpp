#ifndef PFSAMENI_H
#define PFSAMENI_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include "particleSameni.hpp"
#include "utils.hpp"
#include <fstream>



class PFSameni{
public:
    PFSameni(ros::NodeHandle &nh, int numParticles);
    void imageCallback(const std_msgs::Float32::ConstPtr &msg);
    std::vector<ParticleSameni> particles;

    
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
    double computeESS() const;
    double estimate() const;
};

#endif // PFSAMENI_H