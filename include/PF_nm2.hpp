#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <fstream>

struct particle_new2
{
    double A0;
    double omega;
    double zeta;
    double x;
    double v;
    double weight;

    particle_new2(double A0_, double omega_, double zeta_, double x_, double v_, double w_)
        : A0(A0_), omega(omega_), zeta(zeta_), x(x_), v(v_), weight(w_) {}
};

class Particle_new2
{
public:
    Particle_new2(ros::NodeHandle &nh, int numParticles);

private:
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    ros::Publisher pub_;
    std::ofstream log_file_;

    int N_;
    double process_noise_;
    double measurement_noise_;
    double sampling_rate_;

    std::vector<particle_new2> particles;

    void init();
    void predict();
    void updateWeights(double measurement, double measNoiseStd);
    void normalizeWeights();
    void resample();
    double estimate() const;
    double estimateA0() const;
    double estimateOmega() const;
    double estimateZeta() const;
    double computeESS() const;

    void imageCallback(const std_msgs::Float32::ConstPtr &msg);
};
