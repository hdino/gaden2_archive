#ifndef GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED

#include <list>
#include <memory>
#include <random>
//#include <vector>

#include <openvdb/openvdb.h>

#include "filament.hpp"
#include "gas_dispersion_model.hpp"
#include "wind_model.hpp"

namespace gaden {

class EnvironmentModel;

class FilamentModel : public GasDispersionModel
{
public:
    using ConcentrationGrid = openvdb::DoubleGrid;

    FilamentModel(double cell_size, rl::Logger &parent_logger);

    void increment(double time_step);

    inline double getCellSize() const { return cell_size_; }
    const std::list<Filament> & getFilaments() const;

private:
    openvdb::Coord toCoord(const Eigen::Vector3d &p) const;

    ConcentrationGrid::Ptr initConcentrationGrid();

    void addNewFilaments(double time_step);

    void updateGasConcentrationFromFilaments();
    void updateGasConcentration(Filament &filament);

    void updateFilamentLocations(double time_step);
    bool updateFilamentLocation(Filament &filament, double time_step); // returns true if the filament became invalid

    ConcentrationGrid::Ptr concentration_grid_;
    ConcentrationGrid::Accessor concentration_;

    // random
    std::mt19937 random_engine_;
    std::normal_distribution<double> filament_spawn_distribution_;
    std::normal_distribution<double> filament_stochastic_movement_distribution_;

    double cell_size_; // [m] cell_size of the gas distribution
    double cell_num_moles_;
    double filament_num_moles_of_gas_;

    // configuration parameters
    double filament_initial_std_;
    double filament_initial_std_pow2_;
    double filament_spawn_radius_;
    bool variable_filament_rate_; // if true, the number of released filaments will be random (0, num_filaments_per_second_)

    double filament_growth_gamma_;
    double filament_center_concentration_;

    //double gas_gravity_;
    double relative_gas_density_;

    std::list<Filament> filaments_; // use a list because forward_list does not have the 'erase' operation

    // shared_ptrs to other components
    std::shared_ptr<EnvironmentModel> env_model_;
    std::shared_ptr<WindModel> wind_model_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_MODEL_HPP_INCLUDED
