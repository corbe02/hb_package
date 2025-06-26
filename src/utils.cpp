#include "utils.hpp"
#include <random>
#include <cmath>
#include <numeric>

static std::default_random_engine gen(std::random_device{}());
namespace utils
{

    // Funzione per fare il sampling di un numero casuale in una distribuzione gaussiana di media mu e varianza singma
    double sampleNormal(double mu, double sigma)
    {
        std::normal_distribution<double> dist(mu, sigma);
        return dist(gen);
    }

    double sampleUniform(double a, double b)
    {
        std::uniform_real_distribution<> dis(a, b);
        return dis(gen);
    }

    // FUnzione per calcolare la funzione di densità di probabilità (PDF) di una distribuzione normale (gaussiana) in un punto x.
    // La funzione di densità di probabilità
    // descrive come è distribuita la probabilità su una variabile casuale continua.
    // Immagina di avere una curva a campana. L'altezza della curva in un punto x è la densità di probabilità in quel punto.
    double normalPDF(double x, double mu, double sigma)
    {
        double coeff = 1.0 / (sigma * std::sqrt(2.0 * M_PI));
        double exponent = -0.5 * std::pow((x - mu) / sigma, 2);
        return coeff * std::exp(exponent);
    }

    // serve per estrarre N campioni discreti secondo una distribuzione ponderata (non uniforme), usando una discrete distribution
    //  Hai una lista di pesi weights (es. [0.1, 0.3, 0.6]) → ogni peso rappresenta quanto è "probabile" ogni indice.
    //  estraggo solo gli indici non i valori associati
    std::vector<int> sampleDiscrete(const std::vector<double> &weights, int N)
    {
        std::discrete_distribution<int> dist(weights.begin(), weights.end());
        std::vector<int> indices(N);
        for (int i = 0; i < N; ++i)
        {
            indices[i] = dist(gen);
        }
        return indices;
    }

}