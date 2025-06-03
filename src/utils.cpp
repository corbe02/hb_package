#include "utils.hpp"
#include <random>
#include <cmath>
#include <numeric>

static std::default_random_engine gen(std::random_device{}());

double sampleNormal(double mu, double sigma) {
    std::normal_distribution<double> dist(mu, sigma);
    return dist(gen);
}

double normalPDF(double x, double mu, double sigma) {
    double coeff = 1.0 / (sigma * std::sqrt(2.0 * M_PI));
    double exponent = -0.5 * std::pow((x - mu) / sigma, 2);
    return coeff * std::exp(exponent);
}

std::vector<int> sampleDiscrete(const std::vector<double>& weights, int N) {
    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    std::vector<int> indices(N);
    for (int i = 0; i < N; ++i) {
        indices[i] = dist(gen);
    }
    return indices;
}
