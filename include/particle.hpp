#ifndef PARTICLE_HPP
#define PARTICLE_HPP

struct Particle {
    double state;     // Stato stimato
    double weight;    // Peso della particella

    Particle(double s = 0.0, double w = 1.0) : state(s), weight(w) {}
};

#endif
