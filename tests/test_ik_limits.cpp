#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include <cmath>
#include <iostream>

/**
 * @brief Wrap angle to [0, 2pi)
*/
double wrap_to_2pi(double angle)
{
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}

/**
 * @brief Circular sector defined by two angles
*/
struct CircularSector
{
    CircularSector(double first, double second)
    {
        this->first = wrap_to_2pi(first);
        this->second = wrap_to_2pi(second);
    }

    double first;
    double second;
};


void computeIntersection(const CircularSector &sector1,
                         const CircularSector &sector2,
                         std::vector<CircularSector> &intersection)
{
    const double offset = sector1.first;
    // new coordinate system starting at sector1.first
    CircularSector sec1_(0, wrap_to_2pi(sector1.second - offset));
    CircularSector sec2_(wrap_to_2pi(sector2.first - offset), wrap_to_2pi(sector2.second - offset));

    if (sec2_.first < sec2_.second)
    {
        if (sec1_.second < sec1_.first) // no intersection
            return;
        else // one intersection
            intersection.emplace_back(wrap_to_2pi(std::max(sec1_.first, sec2_.first) + offset),
                                      wrap_to_2pi(std::min(sec1_.second, sec2_.second) + offset));
    }
    else
    {
        // at least one intersection
        if (sec2_.first > sec1_.second) // only one intersection
        {
            intersection.emplace_back(wrap_to_2pi(offset), wrap_to_2pi(std::min(sec1_.second, sec2_.second) + offset));
        }
        else // two intersections
        {
            intersection.emplace_back(wrap_to_2pi(offset), wrap_to_2pi(sec2_.second + offset));
            intersection.emplace_back(wrap_to_2pi(sec2_.first + offset), wrap_to_2pi(sec1_.second + offset));
        }
    }
}


void computeIntersection(const std::vector<CircularSector> &sectors1,
                         const std::vector<CircularSector> &sectors2,
                         std::vector<CircularSector> &intersection)
{
    for (const auto &sec1 : sectors1)
    {
        for (const auto &sec2 : sectors2)
        {
            computeIntersection(sec1, sec2, intersection);
        }
    }
}
int main()
{
    // Example usage
    CircularSector sector1 = {-M_PI + 0.2, M_PI};
    CircularSector sector2 = {0.1, M_PI + 0.1};
    std::vector<CircularSector> intersection;

    computeIntersection(sector1, sector2, intersection);

    std::cout << "sector1: (" << sector1.first << "," << sector1.second << ")\n";
    std::cout << "sector2: (" << sector2.first << "," << sector2.second << ")\n";

    for (const auto &sec : intersection)
    {
        std::cout << "Intersection: (" << sec.first << "," << sec.second << ")\n";
    }
    return 0;
}
