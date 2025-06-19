#ifndef PARTICLE_HPP
#define PARTICLE_HPP

// struct Particle {
//     //double state;     // Stato stimato
//     double A;   // ampiezza del battito
//     double f;   // frequenza
//     double phi; // fase corrente
//     double weight;    // Peso della particella

//     Particle(double amp = 0.0, double freq = 0.0, double phase = 0.0,  double w = 1.0) : A(amp), f(freq),phi(phase), weight(w) {}
// };

struct Particle {
    // Stato dinamico
    double x;
    double y;
    double z;

    // Parametri delle onde PQRST
    double a_p, a_q, a_r, a_s, a_t;
    double b_p, b_q, b_r, b_s, b_t;

    // Frequenza angolare
    double omega;

    double weight;

    Particle(
        double x_ = 0.0, double y_ = 1.0, double z_ = 0.0,
        double a_p_ = 1.2, double a_q_ = -5.0, double a_r_ = 30.0, double a_s_ = -7.5, double a_t_ = 0.75,
        double b_p_ = 0.25, double b_q_ = 0.1, double b_r_ = 0.1, double b_s_ = 0.1, double b_t_ = 0.4,
        double omega_ = 2 * M_PI,  // 1 Hz
        double weight_ = 1.0
    )
    : x(x_), y(y_), z(z_),
      a_p(a_p_), a_q(a_q_), a_r(a_r_), a_s(a_s_), a_t(a_t_),
      b_p(b_p_), b_q(b_q_), b_r(b_r_), b_s(b_s_), b_t(b_t_),
      omega(omega_),
      weight(weight_)
    {}
};


#endif
