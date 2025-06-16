#ifndef PARTICLE_HPP
#define PARTICLE_HPP

struct Particle {
    //double state;     // Stato stimato
    double A;   // ampiezza del battito
    double f;   // frequenza
    double phi; // fase corrente
    double weight;    // Peso della particella

    Particle(double amp = 0.0, double freq = 0.0, double phase = 0.0,  double w = 1.0) : A(amp), f(freq),phi(phase), weight(w) {}
};

#endif
