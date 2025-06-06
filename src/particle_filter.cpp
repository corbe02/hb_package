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

ParticleFilter::ParticleFilter(ros::NodeHandle &nh,int numParticles, double initMean, double initStd)
    : private_nh_(nh),N(numParticles), meanInit(initMean), stdInit(initStd) {
    image_sub_ = private_nh_.subscribe("ecg_signal", 1, &ParticleFilter::imageCallback, this);
    particles.resize(N);

    processNoise = 0.05;
    measurementNoise = 0.2;
    log_file.open("/tmp/particle_filter_log.csv");


    init();


    pub = nh.advertise<std_msgs::Float32>("/ecg_denoised", 10);
}


void ParticleFilter::init() {
    for (int i = 0; i < N; ++i) {
        double state = sampleNormal(meanInit, stdInit); //--> valori presi da ecg plotter node su media e deviazione del segnare originale
        particles[i] = Particle(state, 1.0 / N);
    }
}

void ParticleFilter::predict(double processNoise) {
    for (auto& p : particles) {
        // modello semplice: identità + rumore --> non stiamo considerando un modello di ecg 
        // che modello usiamo qui???
        p.state += sampleNormal(0.0, processNoise);
    }
}

void ParticleFilter::updateWeights(double measurement, double measNoiseStd) {
    for (auto& p : particles) {
        // aggiorno i pesi w in base alle misurazioni --> probabilità che una stima fatta con predict sia correttta data la misura 
        double likelihood = normalPDF(measurement, p.state, measNoiseStd); 
        // p.state viene considerata come la media e measNoiseStd come la varianza 
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
    double estimate = 0.0;
    for (const auto& p : particles) {
        estimate += p.state * p.weight;
    }
    return estimate;
}


void ParticleFilter::imageCallback(const std_msgs::Float32::ConstPtr &msg) {
    float z = msg->data;

    // Simile per certi aspetti ai filtri di kalman con i due step principali:
    // prediction --> in cui uso il modello
    // update --> in cui integro con le misurazioni z 

    // Passo 1: prediction (modello di transizione)
    predict(processNoise); 

    // Passo 2: update (basato sulla misura) 
    updateWeights(z, measurementNoise); 

    // Passo 3: normalizzazione
    normalizeWeights();

    // Passo 4: resampling
    resample();

    // Passo 5: stima del segnale pulito
    float est = estimate();


    // salvo il valore in un file così poi farò il plot
    if (log_file.is_open()) {
    log_file << ros::Time::now() << "," << z << "," << estimate() << "\n";
    }

    std_msgs::Float32 out;
    out.data = est;
    pub.publish(out);  
}


