#ifndef GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED

#include <memory>
#include <random>
#include <vector>

#include <openvdb/openvdb.h>

#include "filament.hpp"
#include "gas_dispersion_model.hpp"

namespace gaden {

class EnvironmentModel;

class FilamentModel : public GasDispersionModel
{
public:
    using ConcentrationGrid = openvdb::DoubleGrid;

    FilamentModel(double cell_size, rl::Logger &parent_logger);

    void increment(double time_step);

private:
    openvdb::Coord toCoord(const Eigen::Vector3d &p) const;

    ConcentrationGrid::Ptr initConcentrationGrid();

    void addNewFilaments(double time_step);
    void updateGasConcentrationFromFilaments();
    void updateGasConcentration(Filament &filament);

    ConcentrationGrid::Ptr concentration_grid_;
    ConcentrationGrid::Accessor concentration_;

    // random
    std::mt19937 random_engine_;
    std::normal_distribution<double> filament_spawn_distribution_;

    double cell_num_moles_;
    double filament_num_moles_of_gas_;

    // configuration parameters
    double filament_initial_std_;
    double filament_spawn_radius_;
    bool variable_filament_rate_; // if true, the number of released filaments will be random (0, num_filaments_per_second_)

    std::vector<Filament> filaments_;

    // shared_ptrs to other components
    std::shared_ptr<EnvironmentModel> env_model_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
