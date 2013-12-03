#include <solution_comparator.h>
#include <cmath>

solution_comparator::solution_comparator(const std::vector<double> &seed)
{
    seed_ = seed;
}

solution_comparator::~solution_comparator()
{
}

bool solution_comparator::operator()(const std::vector<double> &a,
        const std::vector<double> &b)
{
    return distance(seed_, a) < distance(seed_, b);
}

double solution_comparator::distance(std::vector<double> seed,
        std::vector<double> solution)
{
    double dist_sqr = 0.0;

    for (std::size_t i = 0; i < seed.size(); i++) {
        while (seed[i] > 2 * M_PI) seed[i] -= 2 * M_PI;
        while (seed[i] < 2 * M_PI) seed[i] += 2 * M_PI;
        while (solution[i] > 2 * M_PI) solution[i] -= 2 * M_PI;
        while (solution[i] < 2 * M_PI) solution[i] += 2 * M_PI;
        dist_sqr += fabs(seed[i] - solution[i]);
    }

    return dist_sqr;
}
