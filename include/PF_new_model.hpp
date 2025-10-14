#ifndef PF_NEW
#define PF_NEW

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <fstream>

// Struttura per rappresentare una particella
struct particle_new
{
    double x;       // posizione
    double v;       // velocità
    double weight;  // peso

    particle_new(double x_, double v_, double w_)
        : x(x_), v(v_), weight(w_) {}
};

class Particle_new
{
public:
    Particle_new(ros::NodeHandle &nh, int numParticles);

private:
    // Parametri del filtro
    int N_;                    // numero di particelle
    double process_noise_;     // rumore di processo (std dev)
    double measurement_noise_; // rumore di misura (std dev)
    double sampling_rate_;     // frequenza di campionamento
    double omega_;             // frequenza naturale (rad/s)
    double zeta_;              // fattore di smorzamento

    ros::NodeHandle private_nh_;

    // Particelle
    std::vector<particle_new> particles;

    // Publisher e subscriber
    ros::Subscriber image_sub_;
    ros::Publisher pub_;

    // File di log
    std::ofstream log_file_;

    // Metodi principali
    void init();
    void predict();
    void updateWeights(double measurement, double measNoiseStd);
    void normalizeWeights();
    void resample();
    double estimate() const;
    double computeESS() const;

    // Callback per i dati in ingresso
    void imageCallback(const std_msgs::Float32::ConstPtr &msg);
};

#endif // PF_NEW
