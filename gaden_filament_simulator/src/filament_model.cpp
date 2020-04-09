#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/simulator.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace gaden {

// gas constant
static constexpr double R = 8.314'462'618'153'24; // [m3 Pa / (K * mol)]
static constexpr double SQRT_8_X_PI_POW3 = std::sqrt(8 * std::pow(M_PI, 3)); // []

FilamentModel::FilamentModel(double cell_size, rl::Logger &parent_logger)
    : GasDispersionModel(cell_size, parent_logger)
    , concentration_grid_(initConcentrationGrid())
    , concentration_(concentration_grid_->getAccessor())
    , random_engine_(std::random_device()())
{
    // load configuration parameters
    double environment_pressure = 1.0 * 101325.0; // [Pa]
    double environment_temperature = 298.0; // [K]

    filament_initial_std_ = 0.1; // [m]
    double ppm_filament_center = 10; // [ppm]

    filament_spawn_radius_ = 1.0;
    variable_filament_rate_ = false; // TODO in config

    //double filament_initial_vol = pow(6 * filament_initial_std, 3); // [m3] -> We approximate the infinite volume of the 3DGaussian as 6 sigmas.
    double cell_vol = std::pow(cell_size, 3); //[m3] volume of a cell
    //	filament_numMoles = (envPressure*filament_initial_vol)/(R*envTemperature);//[mol] Num of moles of Air in that volume
    cell_num_moles_ = (environment_pressure * cell_vol)/(R * environment_temperature); //[mol] Num of moles of Air in that volume

    // The moles of target_gas in a Filament are distributed following a 3D Gaussian
    // Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
    double filament_volume = SQRT_8_X_PI_POW3 // []
                             * std::pow(filament_initial_std_, 3) // [m3]
                             * (ppm_filament_center * 1e-6); // [] --> [m3]
    double moles_per_m3 = environment_pressure / (R * environment_temperature); //[mol/m3]
    filament_num_moles_of_gas_ = filament_volume * moles_per_m3; //[moles_target_gas/filament] This is a CTE parameter!!
}

FilamentModel::ConcentrationGrid::Ptr FilamentModel::initConcentrationGrid()
{
    initialiseOpenVdb();

    // if the background value is not 0, it must be specified here
    ConcentrationGrid::Ptr grid = ConcentrationGrid::create();

    grid->insertMeta("cell_size", openvdb::DoubleMetadata(cell_size));

    return grid;
}

void FilamentModel::increment(double time_step)
{
    env_model_ = simulator->getEnvironmentModel();

    addNewFilaments(time_step);
    updateGasConcentrationFromFilaments();
    //publishMarkers();
    //updateFilamentsLocation();

    env_model_.reset();
}

openvdb::Coord FilamentModel::toCoord(const Eigen::Vector3d &p) const
{
    return openvdb::Coord(p[0] / cell_size,
                          p[1] / cell_size,
                          p[2] / cell_size);
}

void FilamentModel::addNewFilaments(double time_step)
{
    for (const GasSource &gas_source : simulator->getGasSources())
    {
        unsigned num_filaments = time_step * gas_source.num_filaments_per_second;
        unsigned filaments_to_release;
        if (variable_filament_rate_)
        {
            std::uniform_int_distribution<unsigned> filament_amount_distribution(0, num_filaments);
            filaments_to_release = filament_amount_distribution(random_engine_);
        }
        else
            filaments_to_release = num_filaments;

        for (unsigned i = 0; i < filaments_to_release; ++i)
        {
            // Set position of new filament within the specified radius around the gas source location
            Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() { return filament_spawn_distribution_(random_engine_); });
            Eigen::Vector3d filament_position = gas_source.position + vec_random * filament_spawn_radius_;

            Filament filament; // TODO Add constructor to use emplace
            filament.position = filament_position;
            filament.sigma = filament_initial_std_; // TODO Check
            filaments_.push_back(filament);
        }
    }
}

// Here we estimate the gas concentration on each cell of the 3D env
// based on the active filaments and their 3DGaussian shapes
// For that we employ Farrell's Concentration Eq
void FilamentModel::updateGasConcentration(Filament &filament)
{
    // We run over all the active filaments, and update the gas concentration of the cells that are close to them.
    // Ideally a filament spreads over the entire environment, but in practice since filaments are modeled as 3Dgaussians
    // We can stablish a cutt_off raduis of 3*sigma.
    // To avoid resolution problems, we evaluate each filament according to the minimum between:
    // the env_cell_size and filament_sigma. This way we ensure a filament is always well evaluated (not only one point).

    double grid_size = std::min(cell_size, filament.sigma); // [m] grid size to evaluate the filament
        // Compute at which increments the Filament has to be evaluated.
        // If the sigma of the Filament is very big (i.e. the Filament is very flat), the use the world's cell_size.
        // If the Filament is very small (i.e in only spans one or few world cells), then use increments equal to sigma
        //  in order to have several evaluations fall in the same cell.
    double cell_volume = grid_size * grid_size * grid_size; // [m3]

    unsigned num_evaluations = std::ceil(6.0 * filament.sigma / grid_size);
        // How many times the Filament has to be evaluated depends on the final grid_size_m.
        // The filament's grid size is multiplied by 6 because we evaluate it over +-3 sigma
        // If the filament is very small (i.e. grid_size_m = sigma), then the filament is evaluated only 6 times
        // If the filament is very big and spans several cells, then it has to be evaluated for each cell (which will be more than 6)

    // EVALUATE IN ALL THREE AXIS
    double sigma3 = 3.0 * filament.sigma; // [m]
    double sigma_pow2 = filament.sigma * filament.sigma; // [m2]
    double sigma_pow3 = sigma_pow2 * filament.sigma; // [m3]
    double mol_per_m3 = filament_num_moles_of_gas_ / (SQRT_8_X_PI_POW3 * sigma_pow3); // [mol/m3]

    Eigen::Array3d ijk; // should be unsigned, but double avoids a cast
    for (ijk[0] = 0; ijk[0] <= num_evaluations; ++ijk[0])
        for (ijk[1] = 0; ijk[1] <= num_evaluations; ++ijk[1])
            for (ijk[2] = 0; ijk[2] <= num_evaluations; ++ijk[2])
            {
                Eigen::Vector3d p_offset = ijk * grid_size - sigma3; // [m]
                // get point to evaluate
                Eigen::Vector3d p_eval = filament.position + p_offset; // [m]

                // Distance from evaluated_point to filament_center
                double distance_pow2 = p_offset.squaredNorm(); // [m2]

                // FARRELLS Eq.
                //Evaluate the concentration of filament fil_i at given point
                double num_moles_m3 = mol_per_m3 // [mol/m3]
                        * exp(-distance_pow2 / (2 * sigma_pow2)); // [] --> [mol/m3]    TODO Improve variable names

                // Multiply for the volume of the grid cell
                double num_moles = num_moles_m3 * cell_volume; // [mol]

                // Valid point? If either OUT of the environment, or through a wall, treat it as invalid
                if (env_model_->hasObstacleBetweenPoints(filament.position, p_eval))
                {
                    // Point is not valid! Instead of ignoring it, add its concentration to the filament center location
                    // This avoids "loosing" gas concentration as filaments get close to obstacles (e.g. the floor)
                    p_eval = filament.position;
                } // TODO Note that the simulator's behaviour has changed here. In the original code nothing happened if the path is obstructed.

                // Get 3D cell of the evaluated point
                openvdb::Coord cell_coord = toCoord(p_eval);

                // Accumulate concentration in corresponding env_cell
                // TODO Support both ppm and moles as gas concentration unit?

                // ppm version:
                double num_ppm = (num_moles / cell_num_moles_) * 1e6;   //[ppm]
                concentration_.modifyValue(cell_coord, [&](double &val) { val += num_ppm; });
            }
}

void FilamentModel::updateGasConcentrationFromFilaments()
{
    // First, set all cells to 0.0 gas concentration (clear previous state)
    concentration_grid_->clear();

    for (Filament &filament : filaments_)
        updateGasConcentration(filament);
}

} // namespace gaden
