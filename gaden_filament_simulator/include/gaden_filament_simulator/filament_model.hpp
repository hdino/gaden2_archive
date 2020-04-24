#ifndef GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED

#include <list>
#include <memory>
#include <random>
#include <vector>

#include <openvdb/openvdb.h>

#include <gaden_common/gas_source.hpp>

#include "filament.hpp"
#include "gas_dispersion_model.hpp"
#include "wind_model.hpp"

namespace YAML {
class Node;
}

namespace gaden {

namespace chemicals {
class ChemicalBase;
}

template <typename TGrid>
class CacheGrid;

class EnvironmentModel;

class FilamentGasSource : public GasSource
{
public:
    unsigned num_filaments_per_second; // [1/s]
    double mol_per_filament; // [mol]
};

class FilamentModel : public GasDispersionModel
{
public:
    FilamentModel(const YAML::Node &config, rl::Logger &parent_logger);
    ~FilamentModel();

    void increment(double time_step, double total_sim_time);

    const std::list<Filament> & getFilaments() const;

    double getConcentrationAt(const Eigen::Vector3d &position); // returns [ppm]

private:
    void processSimulatorSet();

    void addNewFilaments(double time_step, double total_sim_time);
    void updateFilamentPositions(double time_step, double total_sim_time);

    enum class UpdatePositionResult { Okay, FilamentVanished };
    UpdatePositionResult testAndSetPosition(Eigen::Vector3d &position, const Eigen::Vector3d &candidate);
    UpdatePositionResult updateFilamentPosition(Filament &filament, double time_step, double total_sim_time);

    // random
    //std::mt19937 random_engine_;
    std::default_random_engine random_engine_;
    std::normal_distribution<double> filament_spawn_distribution_;
    std::normal_distribution<double> filament_stochastic_movement_distribution_;

    // configuration parameters
    double filament_initial_radius_;    // [m]
    double filament_growth_gamma_;      // [m2/s]
    double gas_density_factor_;         // [kg/m3]

    std::unique_ptr<chemicals::ChemicalBase> gas_;
    std::vector<FilamentGasSource> gas_sources_;

    std::list<Filament> filaments_;

    // shared_ptrs to other components
    std::shared_ptr<EnvironmentModel> env_model_;
    std::shared_ptr<WindModel> wind_model_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
