#include "ParticleFilter.hpp"
#include "utils.hpp"
#include <fstream>


/*
PARTICLE FILTERS:
Idea:
Voglio stimare una variabile nascosta (x)
Genero N ipotesi (particelle) su quale possa essere x, usando un modello se disponibile, altrimenti distribuendole nello spazio.
Per ogni particella calcolo un peso w che rappresenta quanto bene quella ipotesi spiega l’osservazione (la misura che chiamiamo z).
Poi faccio il *resampling*: prendo N nuove particelle campionando dalle vecchie, in modo che quelle con peso maggiore abbiano più probabilità di essere scelte (→ duplicazioni delle buone, eliminazione delle cattive).
Questo processo viene ripetuto a ogni passo (prediction → update → resampling).
*/




//image_transport::Publisher OpticalFlowPose::image_pub_;

ParticleFilter::ParticleFilter(ros::NodeHandle &nh,int numParticles)
    : private_nh_(nh),N(numParticles){
    particles.resize(N);
    init();
    image_sub_ = private_nh_.subscribe("ecg_signal", 1, &ParticleFilter::imageCallback, this);
    
    processNoise = 0.05;
    measurementNoise = 0.7;
    samplingRate = 60; //Hz

    log_file.open("/tmp/particle_filter_log.csv");
    
    // for (const auto& p : particles) {
    // ROS_INFO_STREAM("init particle: x=" << p.x << ", y=" << p.y << ", z=" << p.z);
    // }

    


    pub = nh.advertise<std_msgs::Float32>("/ecg_denoised", 10);
}

void ParticleFilter::init() {

    for (int i = 0; i < N; ++i) {
        double x = sampleNormal(0.0, 0.1);
        double y = sampleNormal(1.0, 0.1);  
        double z = sampleNormal(0.0, 0.1);

        // Parametri delle onde PQRST: inizializzati attorno ai valori di McSharry
        double a_p = sampleNormal(1.2, 0.01);
        double a_q = sampleNormal(-5.0, 0.02);
        double a_r = sampleNormal(30.0, 0.05);
        double a_s = sampleNormal(-7.5, 0.02);
        double a_t = sampleNormal(0.75, 0.005);

        double b_p = sampleNormal(0.25, 0.002);
        double b_q = sampleNormal(0.1, 0.001);
        double b_r = sampleNormal(0.1, 0.001);
        double b_s = sampleNormal(0.1, 0.001);
        double b_t = sampleNormal(0.4, 0.002);


        double omega = sampleNormal(2 * M_PI, 0.1);


        double weight = 1.0 / N;
        particles[i] = Particle(x, y, z,
                               a_p, a_q, a_r, a_s, a_t,
                               b_p, b_q, b_r, b_s, b_t,
                               omega, weight);
        //ROS_INFO_STREAM("z: " << z << ", x: " << x);
    }
}


void ParticleFilter::predict(double processNoise) {
    double dt = 1.0 / samplingRate;
    for (const auto& p : particles) {
    ROS_INFO_STREAM("predict particle: x=" << p.x << ", y=" << p.y << ", z=" << p.z);
    }

    for (auto& p : particles) {
        //ROS_INFO_STREAM("y: " << p.y << ", x: " << p.x);
        double alpha = 1.0 - std::sqrt(p.x * p.x + p.y * p.y);


        double dx = alpha * p.x - p.omega * p.y;
        double dy = alpha * p.y + p.omega * p.x;

        p.x += dx * dt;
        p.y += dy * dt;

        
        double theta = std::atan2(p.y, p.x);

        double theta_i[5] = { -M_PI / 3, -M_PI / 12, 0, M_PI / 12, M_PI / 2 };
        double a[5]      = { p.a_p, p.a_q, p.a_r, p.a_s, p.a_t };
        double b[5]      = { p.b_p, p.b_q, p.b_r, p.b_s, p.b_t };

        double dz_term = 0.0;
        for (int j = 0; j < 5; ++j) {
            double delta_theta = theta - theta_i[j];
            delta_theta = std::fmod(delta_theta + M_PI, 2 * M_PI);
            if (delta_theta < 0) delta_theta += 2 * M_PI;
            delta_theta -= M_PI;

            dz_term += a[j] * delta_theta * std::exp(-delta_theta * delta_theta / (2 * b[j] * b[j]));
        }

        double dz = -dz_term;

        
        p.z += dz * dt;
        
        // rumore di processo
        p.x += sampleNormal(0.0, processNoise);
        p.y += sampleNormal(0.0, processNoise);
        p.z += sampleNormal(0.0, processNoise);
    }
}


void ParticleFilter::updateWeights(double measurement, double measNoiseStd) {
    for (auto& p : particles) {
        double predicted_z = p.z;
        double likelihood = normalPDF(measurement, predicted_z, measNoiseStd);
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
    newParticles.reserve(N); //// riservo spazio per N elementi (creo un vettore in cui andranno le particelle campionate)

    std::vector<double> weights;
    for (const auto& p : particles) weights.push_back(p.weight); // estraggo i pesi delle particelel (che sono in p.weights) e li metto in weights

    std::vector<int> indices = sampleDiscrete(weights, N); //faccio il sampling di N indici tenendo conto dei weights (probabilità)

    // il samplig è solo degli indici, non so a che valori di particelle corrispondono quindi devo recuperare quello 

    for (int idx : indices) {
        newParticles.push_back(particles[idx]); // metto il contenuto dei rispettivi indici nel vettore delle particelle 
        newParticles.back().weight = 1.0 / N; // reset peso dopo resampling (hanno nuovamente tutte uguale probabilità fino a nuova misurazione)
    }

    particles = std::move(newParticles); //Rimpiazzo il vecchio vettore particles con il nuovo vettore newParticles --> top per ottimizzare 
}

double ParticleFilter::estimate() const {

    // calcolo la media ponderata --> per avere la stima migliore della posizione attuale 
    // dovrebbe migliorare con il tempo 
    // double estimate = 0.0;
    // //for (const auto& p : particles) {
    //     //estimate += p.state * p.weight;
    // //}
    // //return estimate;
    // double estimate_z = 0.0;
    // for (const auto& p : particles) {
    //     //ROS_INFO_STREAM("p.z: " << p.z << ", p.w: " << p.weight);
    //     estimate_z += p.weight * p.z;
    // }
    // return estimate_z;
    // auto best = std::max_element(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
    // return a.weight < b.weight;
    // });
    // return best->z;

        double weight_sum = 0.0;
    for (const auto& p : particles) weight_sum += p.weight;

    double avg_weight = weight_sum / particles.size();

    double robust_estimate = 0.0;
    double robust_weight_sum = 0.0;

    for (const auto& p : particles) {
        if (p.weight >= 0.8 * avg_weight) {  // soglia: metà del peso medio
            robust_estimate += p.weight * p.z;
            robust_weight_sum += p.weight;
        }
    }

    if (robust_weight_sum > 0.0)
        return robust_estimate / robust_weight_sum;
    else
        return 0.0;  // fallback (non dovrebbe succedere spesso)




}


void ParticleFilter::imageCallback(const std_msgs::Float32::ConstPtr &msg) {
    float z = msg->data;

    // Simile per certi aspetti ai filtri di kalman con i due step principali:
    // prediction --> in cui uso il modello
    // update --> in cui integro con le misurazioni z 

    // passo 1: prediction (modello di transizione)
    predict(processNoise); 

    // passo 2: update (basato sulla misura) 
    updateWeights(z, measurementNoise); 

    // passo 3: normalizzazione
    normalizeWeights();

    // Passo 4: resampling
    resample();

    // Passo 5: stima del segnale pulito
    float est = estimate();

    //ROS_INFO_STREAM("z: " << z << ", est: " << est);

    // salvo il valore in un file così poi pplot con plot.py
    if (log_file.is_open()) {
    log_file << ros::Time::now() << "," << z << "," << est << "\n";
    }



    std_msgs::Float32 out;
    out.data = est;
    pub.publish(out);  
}


