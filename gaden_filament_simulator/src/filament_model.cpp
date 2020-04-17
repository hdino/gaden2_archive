#include <gaden_common/cache_grid.hpp>
#include <gaden_common/eigen_helper.hpp> // included to print vectors (for debugging)
#include <gaden_common/grid_helper.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/physical_constants.hpp>
#include <gaden_filament_simulator/simulator.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace gaden {

static constexpr double SQRT_8_X_PI_POW3 = std::sqrt(8 * std::pow(M_PI, 3)); // []

FilamentModel::FilamentModel(const YAML::Node &config, rl::Logger &parent_logger)
    : GasDispersionModel(parent_logger)
    , random_engine_(std::random_device()())
{
    // load configuration parameters
    YAML::Node model_config = config["filament_model"];

    double environment_pressure = 1.0 * 101325.0; // [Pa]
    double environment_temperature = 298.0; // [K]

    filament_initial_std_ = 0.1; // [m]
    filament_initial_std_pow2_ = filament_initial_std_ * filament_initial_std_;

    double filament_noise_std = 0.1; // [m] Sigma of the white noise added on each iteration
    filament_stochastic_movement_distribution_ = std::normal_distribution<double>(0, filament_noise_std);

    filament_spawn_radius_ = 1.0; // [m]
    variable_filament_rate_ = false; // TODO in config

    filament_growth_gamma_ = 0.01; // [m2/s]
    filament_center_concentration_ = 20e-6; // [] --> ppm * 1e-6

    double cell_size = model_config["cell_size"].as<double>();

    // compute derived parameters
    double gas_specific_gravity = constant::specific_gravity::methane; // []
    relative_gas_density_ = constant::density_air * (constant::specific_gravity::air - gas_specific_gravity); // [kg/m3]

    //double filament_initial_vol = pow(6 * filament_initial_std, 3); // [m3] -> We approximate the infinite volume of the 3DGaussian as 6 sigmas.
    //double cell_vol = std::pow(cell_size, 3); //[m3] volume of a cell
    //	filament_numMoles = (envPressure*filament_initial_vol)/(R*envTemperature);//[mol] Num of moles of Air in that volume
    //double num_moles_per_m3 = environment_pressure / (constant::R * environment_temperature); // [mol/m3]
    //cell_num_moles_ = cell_vol * num_moles_per_m3; //[mol] Num of moles of Air in that volume

    // The moles of target_gas in a Filament are distributed following a 3D Gaussian
    // Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
    double filament_volume = SQRT_8_X_PI_POW3 // []
                             * std::pow(filament_initial_std_, 3) // [m3]
                             * filament_center_concentration_; // [] --> [m3]
    double moles_per_m3 = environment_pressure / (constant::R * environment_temperature); //[mol/m3]
    filament_num_moles_of_gas_ = filament_volume * moles_per_m3; //[moles_target_gas/filament] This is a CTE parameter!!

    // print configuration information
    logger.info() << "Gas dispersion model: Filament model"
                  << "\n  Cell size: " << cell_size;

    // concentration cache
    concentration_cache_ = std::make_unique<CacheGrid<openvdb::DoubleGrid>>(
                cell_size,
                [this](auto&& ...x) {
                    return computeConcentrationAt(std::forward<decltype(x)>(x)...); });
}

FilamentModel::~FilamentModel()
{}

const std::list<Filament> & FilamentModel::getFilaments() const
{
    return filaments_;
}

void FilamentModel::increment(double time_step, double total_sim_time)
{
    env_model_ = simulator->getEnvironmentModel();
    wind_model_ = simulator->getWindModel();

    addNewFilaments(time_step, total_sim_time);
    updateFilamentPositions(time_step, total_sim_time);
    concentration_cache_->clear();

    env_model_.reset();
    wind_model_.reset();
}

void FilamentModel::addNewFilaments(double time_step, double total_sim_time)
{
    for (const GasSource &gas_source : simulator->getGasSources())
    {
        // TODO Make sure that the number of filaments is met for a period of several seconds?
        unsigned num_filaments = time_step * gas_source.num_filaments_per_second;
        unsigned filaments_to_release;
        if (variable_filament_rate_)
        {
            std::uniform_int_distribution<unsigned> filament_amount_distribution(0, num_filaments);
            filaments_to_release = filament_amount_distribution(random_engine_);
        }
        else
            filaments_to_release = num_filaments;

        //logger.info() << "Adding " << filaments_to_release << " new filaments for gas source at " << toString(gas_source.position);

        for (unsigned i = 0; i < filaments_to_release; ++i)
        {
            // Set position of new filament within the specified radius around the gas source location
            Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
                return filament_spawn_distribution_(random_engine_); });

            Eigen::Vector3d filament_position = gas_source.position + vec_random * filament_spawn_radius_;

            filaments_.emplace_back(filament_position,
                                    filament_initial_std_, // TODO Check if filament_initial_std_ is correct here
                                    total_sim_time);
        }
    }

    //logger.info() << "Filaments after addNewFilaments: " << filaments_.size();
}

void FilamentModel::updateFilamentPositions(double time_step, double total_sim_time)
{
    for (auto it = filaments_.begin(); it != filaments_.end();)
    {
        if (updateFilamentPosition(*it, time_step, total_sim_time) == UpdatePositionResult::FilamentVanished)
            it = filaments_.erase(it);
        else
            ++it;
    }
}

FilamentModel::UpdatePositionResult
FilamentModel::testAndSetPosition(Eigen::Vector3d &position,
                                  const Eigen::Vector3d &candidate)
{
    switch (env_model_->getOccupancy(candidate))
    {
    case Occupancy::Free:
        // Free and valid location... update filament position
        position = candidate;
        return UpdatePositionResult::Okay;
    case Occupancy::Outlet:
        // The location corresponds to an outlet! Delete filament!
        //logger.info() << "Filament reached outlet at " << toString(candidate);
        return UpdatePositionResult::FilamentVanished;
    default:
        // The location falls in an obstacle --> illegal movement, do not apply it
        return UpdatePositionResult::Okay;
    }
}

//Update the filaments location in the 3D environment
// According to Farrell Filament model, a filament is afected by three components of the wind flow.
// 1. Va (large scale wind) -> Advection (Va) -> Movement of a filament as a whole by wind) -> from CFD
// 2. Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as white noise
// 3. Vd (small scale wind) -> Difussion or change of the filament shape (growth with time)
// We also consider Gravity and Bouyant Forces given the gas molecular mass
FilamentModel::UpdatePositionResult
FilamentModel::updateFilamentPosition(Filament &filament, double time_step, double total_sim_time)
{
    Eigen::Vector3d new_position;

    // 1. Simulate Advection (Va)
    // Large scale wind-eddies -> Movement of a filament as a whole by wind
    // --------------------------------------------------------------------
    new_position = filament.position + time_step * wind_model_->getWindVelocityAt(filament.position);
    //logger.info() << "Position after advection: " << toString(new_position);

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 2. Simulate Gravity & Bouyant Force
    // -----------------------------------

    // Approximation from "Terminal Velocity of a Bubble Rise in a Liquid Column",
    // World Academy of Science, Engineering and Technology 28 2007
    double terminal_buoyancy_velocity_z = // TODO *d^2 is missing (d = bubble diameter)
            (constant::g * relative_gas_density_ * filament_center_concentration_) // [m/s2] * [kg/m3] * [] = [kg/s2m2]
            / (18.0 * constant::dynamic_viscosity_air); // [kg/(m*s)] --> [1/(m*s)]
    Eigen::Vector3d terminal_buoyancy_velocity(
                0,
                0,
                terminal_buoyancy_velocity_z);

    new_position = filament.position + terminal_buoyancy_velocity * time_step;
    //logger.info() << "Position after gravity and bouyant force: " << toString(new_position);

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 3. Add some variability (stochastic process)
    // Vm (middle scale wind)-> Movement of the filament with respect to the
    // center of the "plume" -> modeled as Gaussian white noise
    // ---------------------------------------------------------------------
    Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
        return filament_stochastic_movement_distribution_(random_engine_); });

    new_position = filament.position + vec_random;
    //logger.info() << "Position after stochastic process: " << toString(new_position);

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 4. Filament growth with time (this affects the posterior estimation of
    //                               gas concentration at each cell)
    // Vd (small scale wind eddies) --> Difussion or change of the filament
    //                                  shape (growth with time)
    // R = sigma of a 3D gaussian --> Increasing sigma with time
    // ------------------------------------------------------------------------
    double filament_age = total_sim_time - filament.birth_time; // [s]
    filament.sigma = std::sqrt(filament_initial_std_pow2_ + filament_growth_gamma_ * filament_age);
    //           m =      sqrt([m2]                       + [m2/s]                 * [s]         )

    return UpdatePositionResult::Okay;
}

double FilamentModel::getConcentrationAt(const Eigen::Vector3d &position)
{
    return concentration_cache_->getValue(position);
}

double FilamentModel::computeConcentrationAt(const Eigen::Vector3d &position)
{
    return 1;
}

//// Here we estimate the gas concentration on each cell of the 3D env
//// based on the active filaments and their 3DGaussian shapes
//// For that we employ Farrell's Concentration Eq
//void FilamentModel::updateGasConcentration(Filament &filament)
//{
//    // We run over all the active filaments, and update the gas concentration of the cells that are close to them.
//    // Ideally a filament spreads over the entire environment, but in practice since filaments are modeled as 3Dgaussians
//    // We can stablish a cutt_off raduis of 3*sigma.
//    // To avoid resolution problems, we evaluate each filament according to the minimum between:
//    // the env_cell_size and filament_sigma. This way we ensure a filament is always well evaluated (not only one point).

//    double grid_size = std::min(cell_size_, filament.sigma); // [m] grid size to evaluate the filament
//        // Compute at which increments the Filament has to be evaluated.
//        // If the sigma of the Filament is very big (i.e. the Filament is very flat), the use the world's cell_size.
//        // If the Filament is very small (i.e in only spans one or few world cells), then use increments equal to sigma
//        //  in order to have several evaluations fall in the same cell.
//    double cell_volume = grid_size * grid_size * grid_size; // [m3]

//    unsigned num_evaluations = std::ceil(6.0 * filament.sigma / grid_size);
//        // How many times the Filament has to be evaluated depends on the final grid_size_m.
//        // The filament's grid size is multiplied by 6 because we evaluate it over +-3 sigma
//        // If the filament is very small (i.e. grid_size_m = sigma), then the filament is evaluated only 6 times
//        // If the filament is very big and spans several cells, then it has to be evaluated for each cell (which will be more than 6)

//    // EVALUATE IN ALL THREE AXIS
//    double sigma3 = 3.0 * filament.sigma; // [m]
//    double sigma_pow2 = filament.sigma * filament.sigma; // [m2]
//    double sigma_pow3 = sigma_pow2 * filament.sigma; // [m3]
//    double mol_per_m3 = filament_num_moles_of_gas_ / (SQRT_8_X_PI_POW3 * sigma_pow3); // [mol/m3]

//    Eigen::Array3d ijk; // should be unsigned, but double avoids a cast
//    for (ijk[0] = 0; ijk[0] <= num_evaluations; ++ijk[0])
//        for (ijk[1] = 0; ijk[1] <= num_evaluations; ++ijk[1])
//            for (ijk[2] = 0; ijk[2] <= num_evaluations; ++ijk[2])
//            {
//                Eigen::Vector3d p_offset = ijk * grid_size - sigma3; // [m]
//                // get point to evaluate
//                Eigen::Vector3d p_eval = filament.position + p_offset; // [m]

//                // Distance from evaluated_point to filament_center
//                double distance_pow2 = p_offset.squaredNorm(); // [m2]

//                // FARRELLS Eq.
//                //Evaluate the concentration of filament fil_i at given point
//                double num_moles_m3 = mol_per_m3 // [mol/m3]
//                        * exp(-distance_pow2 / (2 * sigma_pow2)); // [] --> [mol/m3]    TODO Improve variable names

//                // Multiply for the volume of the grid cell
//                double num_moles = num_moles_m3 * cell_volume; // [mol]

//                // Valid point? If either OUT of the environment, or through a wall, treat it as invalid
//                if (env_model_->hasObstacleBetweenPoints(filament.position, p_eval))
//                {
//                    // Point is not valid! Instead of ignoring it, add its concentration to the filament center location
//                    // This avoids "loosing" gas concentration as filaments get close to obstacles (e.g. the floor)
//                    p_eval = filament.position;
//                } // TODO Note that the simulator's behaviour has changed here. In the original code nothing happened if the path is obstructed.

//                // Get 3D cell of the evaluated point
//                openvdb::Coord cell_coord = grid_helper::getCellCoordinates(p_eval, cell_size_); //toCoord(p_eval);

//                // Accumulate concentration in corresponding env_cell
//                // TODO Support both ppm and moles as gas concentration unit?

//                // ppm version:
//                double num_ppm = (num_moles / cell_num_moles_) * 1e6;   //[ppm]
//                concentration_.modifyValue(cell_coord, [&](double &val) { val += num_ppm; });
//            }
//}

//void FilamentModel::updateGasConcentrationFromFilaments()
//{
//    // First, set all cells to 0.0 gas concentration (clear previous state)
//    concentration_grid_->clear();

//    for (Filament &filament : filaments_)
//        updateGasConcentration(filament);
//}

} // namespace gaden
