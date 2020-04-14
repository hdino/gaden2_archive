#ifndef GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED

#include <Eigen/Core>

namespace gaden {

struct Filament
{
    Filament() : age(0) {}

    Eigen::Vector3d position; // [m] center of the filament
    double sigma; // [m] The sigma of a 3D gaussian (controlls the shape of the filament)
    double age; // [s]
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
