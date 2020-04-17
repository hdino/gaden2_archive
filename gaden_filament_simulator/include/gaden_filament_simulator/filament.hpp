#ifndef GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED

#include <Eigen/Core>

namespace gaden {

struct Filament
{
    Filament(Eigen::Vector3d init_position,
             double init_sigma,
             double init_time)
        : position(init_position)
        , sigma(init_sigma)
        , birth_time(init_time)
    {}

    Eigen::Vector3d position; // [m] center of the filament
    double sigma;             // [m] The sigma of a 3D gaussian
                              //     (controls the shape of the filament)
    double birth_time;        // [s] Time at which the filament is released
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
