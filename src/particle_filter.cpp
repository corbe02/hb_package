#include "ParticleFilter.hpp"
#include "utils.hpp"


//image_transport::Publisher OpticalFlowPose::image_pub_;

ParticleFilter::ParticleFilter(ros::NodeHandle &nh,int numParticles, double initMean, double initStd)
    : private_nh_(nh),N(numParticles), meanInit(initMean), stdInit(initStd) {
    image_sub_ = private_nh_.subscribe("ecg_signal", 1, &ParticleFilter::imageCallback, this);
    particles.resize(N);

    processNoise = 0.05;
    measurementNoise = 0.2;

    init();


    pub = nh.advertise<std_msgs::Float32>("/ecg_denoised", 10);
}


void ParticleFilter::init() {
    for (int i = 0; i < N; ++i) {
        double state = sampleNormal(meanInit, stdInit);
        particles[i] = Particle(state, 1.0 / N);
    }
}

void ParticleFilter::predict(double processNoise) {
    for (auto& p : particles) {
        // modello semplice: identità + rumore
        p.state += sampleNormal(0.0, processNoise);
    }
}

void ParticleFilter::updateWeights(double measurement, double measNoiseStd) {
    for (auto& p : particles) {
        // peso basato sulla probabilità della misura dato lo stato
        double likelihood = normalPDF(measurement, p.state, measNoiseStd);
        p.weight *= likelihood;
    }
}

void ParticleFilter::normalizeWeights() {
    double sum = 0.0;
    for (const auto& p : particles) sum += p.weight;
    if (sum == 0) {
        for (auto& p : particles) p.weight = 1.0 / N;
    } else {
        for (auto& p : particles) p.weight /= sum;
    }
}

void ParticleFilter::resample() {
    std::vector<Particle> newParticles;
    newParticles.reserve(N);

    std::vector<double> weights;
    for (const auto& p : particles) weights.push_back(p.weight);

    std::vector<int> indices = sampleDiscrete(weights, N);

    for (int idx : indices) {
        newParticles.push_back(particles[idx]);
        newParticles.back().weight = 1.0 / N; // reset peso dopo resampling
    }

    particles = std::move(newParticles);
}

double ParticleFilter::estimate() const {
    double estimate = 0.0;
    for (const auto& p : particles) {
        estimate += p.state * p.weight;
    }
    return estimate;
}


void ParticleFilter::imageCallback(const std_msgs::Float32::ConstPtr &msg) {
    float z = msg->data;

    // Passo 1: prediction (modello di transizione)
    predict(processNoise);  // es: 0.05

    // Passo 2: update (basato sulla misura ricevuta)
    updateWeights(z, measurementNoise);  // es: 0.2

    // Passo 3: normalizzazione
    normalizeWeights();

    // Passo 4: resampling
    resample();

    // Passo 5: stima del segnale pulito
    float est = estimate();

    // (Opzionale) Pubblica il valore stimato
    std_msgs::Float32 out;
    out.data = est;
    pub.publish(out);  
}


