#ifndef SOLUTION_COMPARATOR_H
#define SOLUTION_COMPARATOR_H

#include <vector>

class solution_comparator
{
    public:
        /**
         * Ctor.
         */
        solution_comparator(const std::vector<double> &seed);

        /**
         * Dtor.
         */
        virtual ~solution_comparator();

        /**
         * Compare two solutions.
         *
         * @return true when a is less than b (a < b), false otherwise.
         */
        bool operator()(const std::vector<double> &a,
                const std::vector<double> &b);

    private:
        /**
         * Calculate the distance between the given seed state and a solution.
         */
        double distance(std::vector<double> seed, std::vector<double> solution);

    private:
        std::vector<double> seed_;
};

#endif
