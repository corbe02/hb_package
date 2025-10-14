#include "PF_new_model.hpp"
#include "utils.hpp"
#include <cmath>
#include <algorithm>

Particle_new::Particle_new(ros::NodeHandle &nh, int numParticles)
    : private_nh_(nh),
      N_(numParticles),
      process_noise_(0.001),
      measurement_noise_(0.005),
      sampling_rate_(60.0),
      omega_(2 * M_PI * 1.5), // frequenza naturale (≈ 1.5 Hz)
      zeta_(0.1)              // smorzamento leggero
{
    init();

    image_sub_ = private_nh_.subscribe("ecg_signal", 1, &Particle_new::imageCallback, this);

    log_file_.open("/tmp/particle_filter_mech.csv", std::ios::out);
    if (!log_file_.is_open())
    {

        ROS_ERROR("Impossibile aprire il file di log!");
    }
    else
    {
        ROS_INFO_STREAM("File aperto");
        log_file_ << "time,measurement,estimate\n";
    }

    pub_ = nh.advertise<std_msgs::Float32>("/ecg_denoised", 1);
}

void Particle_new::init()
{
    particles.clear();
    particles.reserve(N_);

    for (int i = 0; i < N_; ++i)
    {
        particles.emplace_back(
            utils::sampleNormal(0.0, 0.01),   // posizione iniziale x
            utils::sampleNormal(0.0, 0.01),   // velocità iniziale v
            1.0 / N_                          // peso
        );
    }
}

void Particle_new::predict()
{
    double dt = 1.0 / sampling_rate_;
    static double t = 0.0;

    // Forzante sinusoidale (movimento cardiaco)
    double A0 = 0.1;                  // ampiezza forza
    double omega_f = 2 * M_PI * 1;  // 1 Hz → battito cardiaco
    double phi = 0.0;                 // fase iniziale

    for (auto &p : particles)
    {
        double x = p.x;
        double v = p.v;

        // accelerazione secondo modello viscoelastico
        double a = -2.0 * zeta_ * omega_ * v - omega_ * omega_ * x + A0 * sin(omega_f * t + phi);

        // integrazione (Eulero esplicito)
        double new_x = x + dt * v;
        double new_v = v + dt * a;

        // aggiungo rumore di processo
        new_x += utils::sampleNormal(0.0, process_noise_);
        new_v += utils::sampleNormal(0.0, process_noise_);

        // aggiorno
        p.x = new_x;
        p.v = new_v;
    }

    t += dt;
}

void Particle_new::updateWeights(double measurement, double measNoiseStd)
{
    for (auto &p : particles)
    {
        double predicted_z = p.x; // osserviamo direttamente la posizione
        double likelihood = utils::normalPDF(measurement, predicted_z, measNoiseStd);
        p.weight *= likelihood;
    }
}

void Particle_new::normalizeWeights()
{
    double sum = 0.0;
    for (const auto &p : particles)
        sum += p.weight;

    if (sum == 0.0)
    {
        for (auto &p : particles)
            p.weight = 1.0 / N_;
    }
    else
    {
        for (auto &p : particles)
            p.weight /= sum;
    }
}

void Particle_new::resample()
{
    std::vector<particle_new> newParticles;
    newParticles.reserve(N_);

    std::vector<double> weights;
    for (const auto &p : particles)
        weights.push_back(p.weight);

    std::vector<int> indices = utils::sampleDiscrete(weights, N_);

    for (int idx : indices)
    {
        newParticles.push_back(particles[idx]);
        newParticles.back().weight = 1.0 / N_;
    }

    particles = std::move(newParticles);
}

double Particle_new::estimate() const
{
    double est = 0.0;
    for (const auto &p : particles)
        est += p.weight * p.x;
    return est;
}

double Particle_new::computeESS() const
{
    double sumsq = 0.0;
    for (const auto &p : particles)
        sumsq += p.weight * p.weight;
    return 1.0 / sumsq;
}

void Particle_new::imageCallback(const std_msgs::Float32::ConstPtr &msg)
{
    double z = msg->data;

    predict();
    updateWeights(z, measurement_noise_);
    normalizeWeights();

    double ess = computeESS();
    if (ess < 0.5 * N_)
        resample();

    double est = estimate();

    if (log_file_.is_open())
    {
        log_file_ << ros::Time::now() << "," << z << "," << est << "\n";
    }

    std_msgs::Float32 out;
    out.data = est;
    pub_.publish(out);
}
