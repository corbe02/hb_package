#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>

namespace utils
{
    // Campiona un numero da una normale N(mu, sigma)
    double sampleNormal(double mu, double sigma);

    double sampleUniform(double a, double b);

    // Valuta la densità della normale N(mu, sigma) in x
    double normalPDF(double x, double mu, double sigma);

    // Resampling: restituisce N indici da una distribuzione discreta data dai pesi
    std::vector<int> sampleDiscrete(const std::vector<double> &weights, int N);

    double studentTPDF(double x, double mu, double sigma, double nu = 5.0);

}

#endif
