#include "PF_nm2.hpp"
#include "utils.hpp"
#include <cmath>
#include <algorithm>

Particle_new2::Particle_new2(ros::NodeHandle &nh, int numParticles)
    : private_nh_(nh),
      N_(numParticles),
      process_noise_(0.01),
      measurement_noise_(0.3),
      sampling_rate_(60.0)
{
    init();

    image_sub_ = private_nh_.subscribe("ecg_signal", 1, &Particle_new2::imageCallback, this);

    log_file_.open("/tmp/particle_filter_mech.csv", std::ios::out);
    if (!log_file_.is_open())
    {
        ROS_ERROR("Impossibile aprire il file di log!");
    }
    else
    {
        ROS_INFO_STREAM("File aperto per logging");
        log_file_ << "time,measurement,estimate_x,est_A0,est_omega,est_zeta\n";
    }

    pub_ = nh.advertise<std_msgs::Float32>("/ecg_denoised", 1);
}

/* ============================================================
   Inizializzazione delle particelle
   Ogni particella ha i propri parametri (A0, omega, zeta)
   e uno stato dinamico (x, v)
   ============================================================ */
void Particle_new2::init()
{
    particles.clear();
    particles.reserve(N_);

    for (int i = 0; i < N_; ++i)
    {
        particles.emplace_back(
            utils::sampleNormal(80.0, 20.0),  // A0 iniziale più grande
            utils::sampleNormal(2 * M_PI * 1.5, 0.2), 
            utils::sampleNormal(0.05, 0.01),
            utils::sampleNormal(0.0, 0.01),
            utils::sampleNormal(0.0, 0.01),
            1.0 / N_                             // peso iniziale
        );
    }
}

/* ============================================================
   Predizione: evoluzione del modello fisico
   Ogni particella evolve in base ai propri parametri.
   ============================================================ */
void Particle_new2::predict()
{
    double dt = 1.0 / sampling_rate_;
    static double t = 0.0;
    double phi = 0.0;

    for (auto &p : particles)
    {
        // piccola evoluzione dei parametri
        p.A0 += utils::sampleNormal(0.0, 0.1);
        p.omega += utils::sampleNormal(0.0, 0.02);
        p.zeta += utils::sampleNormal(0.0, 0.002);

        // vincoli fisici
        p.A0 = std::max(0.0, std::min(p.A0, 100.0));
        p.omega = std::max(2*M_PI*0.5, std::min(p.omega, 2*M_PI*3.0));
        p.zeta = std::max(0.0, std::min(p.zeta, 0.3));

        // accelerazione: usare omega della particella stessa
        double a = -2.0 * p.zeta * p.omega * p.v
                   - p.omega * p.omega * p.x
                   + p.A0 * sin(p.omega * t + phi);

        // integrazione (Eulero)
        p.x += dt * p.v;
        p.v += dt * a;

        // rumore ridotto
        p.x += utils::sampleNormal(0.0, 0.01);
        p.v += utils::sampleNormal(0.0, 0.01);
    }

    t += dt;
}

/* ============================================================
   Aggiornamento dei pesi: confronto misura vs posizione stimata
   ============================================================ */
void Particle_new2::updateWeights(double measurement, double measNoiseStd)
{
    for (auto &p : particles)
    {
        double predicted_z = p.x; // osserviamo direttamente la posizione
        double likelihood = utils::normalPDF(measurement, predicted_z, measNoiseStd);
        p.weight *= likelihood;
    }
}

/* ============================================================
   Normalizzazione dei pesi
   ============================================================ */
void Particle_new2::normalizeWeights()
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

/* ============================================================
   Resampling sistematico
   ============================================================ */
void Particle_new2::resample()
{
    std::vector<particle_new2> newParticles;
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

/* ============================================================
   Stima media pesata della posizione
   ============================================================ */
double Particle_new2::estimate() const
{
    double est = 0.0;
    for (const auto &p : particles)
        est += p.weight * p.x;
    return est;
}

/* ============================================================
   Stima media dei parametri (per logging o debug)
   ============================================================ */
double Particle_new2::estimateA0() const
{
    double e = 0.0;
    for (const auto &p : particles)
        e += p.weight * p.A0;
    return e;
}

double Particle_new2::estimateOmega() const
{
    double e = 0.0;
    for (const auto &p : particles)
        e += p.weight * p.omega;
    return e;
}

double Particle_new2::estimateZeta() const
{
    double e = 0.0;
    for (const auto &p : particles)
        e += p.weight * p.zeta;
    return e;
}

/* ============================================================
   Calcolo dell'ESS (Effective Sample Size)
   ============================================================ */
double Particle_new2::computeESS() const
{
    double sumsq = 0.0;
    for (const auto &p : particles)
        sumsq += p.weight * p.weight;
    return 1.0 / sumsq;
}

/* ============================================================
   Callback ROS: aggiornamento e pubblicazione
   ============================================================ */
void Particle_new2::imageCallback(const std_msgs::Float32::ConstPtr &msg)
{
    double z = msg->data;

    predict();
    updateWeights(z, measurement_noise_);
    normalizeWeights();

    double ess = computeESS();
    if (ess < 0.5 * N_)
        resample();

    double est_x = estimate();
    double estA0 = estimateA0();
    double estOmega = estimateOmega();
    double estZeta = estimateZeta();

    if (log_file_.is_open())
    {
        log_file_ << ros::Time::now() << ","
                  << z << ","
                  << est_x << ","
                  << estA0 << ","
                  << estOmega << ","
                  << estZeta << "\n";
    }

    // pubblica solo la posizione stimata
    std_msgs::Float32 out;
    out.data = est_x;
    pub_.publish(out);
}
