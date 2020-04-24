#include <gaden_common/chemical_substance.hpp>
#include <gaden_common/eigen_helper.hpp> // included to print vectors (for debugging)
//#include <gaden_common/grid_helper.hpp>
//#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/physical_constants.hpp>
#include <gaden_filament_simulator/simulator.hpp>

//#include <algorithm>
//#include <cmath>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace gaden {

//static constexpr double PI_POW3 = M_PI * M_PI * M_PI;
//static constexpr double SQRT_8_X_PI_POW3 = std::sqrt(8 * PI_POW3); // []
static constexpr double IDEAL_GAS_M3_PER_MOL = constant::R * constant::toKelvinFromDegreeCelsius(0) / constant::toPascalFromAtmosphere(1);
static constexpr double IDEAL_GAS_MOL_PER_M3 = 1.0 / IDEAL_GAS_M3_PER_MOL;

FilamentModel::FilamentModel(const YAML::Node &config, rl::Logger &parent_logger)
    : GasDispersionModel(parent_logger)
    , random_engine_(std::random_device()())
{
    // load configuration parameters
    YAML::Node model_config = config["filament_model"];

    double filament_spawn_radius = 1.0; // [m]
        // radius around the source in which the filaments are spawned
    double filament_concentration = 20e-6; // [], e.g. 20 ppm --> 20e-6

    filament_initial_radius_ = 0.1; // [m], R(0) in Farrell's paper
    filament_growth_gamma_ = 0.01; // [m2/s]
        // gamma that controls the rate of growth in Farrell's paper

    double filament_noise_std = 0.1; // [m]
        // Sigma of the white noise added to the filament's position
        // on each iteration

    gas_ = std::make_unique<chemicals::Methane>();

    // Create random distributions
    filament_spawn_distribution_ = std::normal_distribution<double>(0, filament_spawn_radius / 3.0);
        // Set standard deviation to spawn_radius/3, such that 99.73%
        // of all filaments will spawn within that radius
    filament_stochastic_movement_distribution_ = std::normal_distribution<double>(0, filament_noise_std);

    // Compute the gas density factor: (rho_air - rho_gas) * concentration
    // This is not an established number, but we multiply by the
    // concentration here to save computing time later.
    double gas_density_delta = chemicals::Air::MassDensity() - gas_->getMassDensity(); // [kg/m3]
    gas_density_factor_ = gas_density_delta * filament_concentration; // [kg/m3]
    // TODO Make the mass density depend on temperature (assuming pressure is always 1 atm)

    //double environment_pressure = 1.0 * 101325.0; // [Pa]
    //double environment_temperature = 290.0; // [K]


    //double filament_initial_vol = pow(6 * filament_initial_std, 3); // [m3] -> We approximate the infinite volume of the 3DGaussian as 6 sigmas.
    //double cell_vol = std::pow(cell_size, 3); //[m3] volume of a cell
    //	filament_numMoles = (envPressure*filament_initial_vol)/(R*envTemperature);//[mol] Num of moles of Air in that volume
    //double num_moles_per_m3 = environment_pressure / (constant::R * environment_temperature); // [mol/m3]
    //cell_num_moles_ = cell_vol * num_moles_per_m3; //[mol] Num of moles of Air in that volume

    // The moles of target_gas in a Filament are distributed following a 3D Gaussian
    // Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
//    double filament_volume = SQRT_8_X_PI_POW3 // []
//                             * std::pow(filament_initial_radius_, 3) // [m3]
//                             * filament_center_concentration_; // [] --> [m3]
//    double moles_per_m3 = environment_pressure / (constant::R * environment_temperature); //[mol/m3]
//    filament_num_moles_of_gas_ = filament_volume * moles_per_m3; //[moles_target_gas/filament] This is a CTE parameter!!

    // print configuration information
    logger.info() << "Gas dispersion model: Filament model";
}

void FilamentModel::processSimulatorSet()
{
    env_model_ = simulator->getEnvironmentModel();
    wind_model_ = simulator->getWindModel();

    // Compute filament release rate of each gas source
    for (const GasSource &gas_source : simulator->getGasSources())
    {
        FilamentGasSource filament_gas_source;
        static_cast<GasSource &>(filament_gas_source) = gas_source;

        double molar_release_rate = gas_source.release_rate
                                    / gas_->getMolarMass()
                                    / 3600.0;
                                    // kg/h / (kg/mol) / (s/h) = mol/s

//        double molecules_release_rate = molar_release_rate * constant::N_Avogadro;
//            // mol/s * mol^-1 = 1/s

        filament_gas_source.num_filaments_per_second = 10; // [1/s]
        filament_gas_source.mol_per_filament =
                molar_release_rate / filament_gas_source.num_filaments_per_second;
        gas_sources_.push_back(filament_gas_source);

        logger.info() << "Gas source at " << toString(filament_gas_source.position)
                      << " release_rate: " << std::to_string(filament_gas_source.num_filaments_per_second) << " filaments/s"
                      << " mol/filament: " << std::to_string(filament_gas_source.mol_per_filament);
    }
}

FilamentModel::~FilamentModel()
{}

const std::list<Filament> & FilamentModel::getFilaments() const
{
    return filaments_;
}

double FilamentModel::getConcentrationAt(const Eigen::Vector3d &position)
{
    double concentration = 0; // [mol/m3]

    for (const Filament &filament : filaments_)
    {
        if (env_model_->hasObstacleBetweenPoints(position, filament.position))
            continue;
        concentration += filament.getConcentrationAt(position);
    }

    // convert mol/m3 to ppm
    double air_concentration = IDEAL_GAS_MOL_PER_M3 - concentration;
    if (air_concentration < 1e-3)
    {
        logger.warn() << "Air concentration below 1e-3: " << air_concentration;
        air_concentration = 1e-3;
    }

    double concentration_ppm = concentration / air_concentration * 1e6;

    return concentration_ppm;
}

void FilamentModel::increment(double time_step, double total_sim_time)
{
    addNewFilaments(time_step, total_sim_time);
    updateFilamentPositions(time_step, total_sim_time);
}

void FilamentModel::addNewFilaments(double time_step, double total_sim_time)
{
    (void)total_sim_time;

    for (const FilamentGasSource &gas_source : gas_sources_)
    {
        // TODO Make sure that the number of filaments is met for a period of several seconds?
        unsigned num_filaments = time_step * gas_source.num_filaments_per_second;
        unsigned filaments_to_release = num_filaments;
//        if (gas_source.variable_release_rate)
//        {
//            std::poisson_distribution<unsigned> filament_amount_distribution(num_filaments);
//            filaments_to_release = filament_amount_distribution(random_engine_);
//        }
//        else
//            filaments_to_release = num_filaments;

        for (unsigned i = 0; i < filaments_to_release; ++i)
        {
            // Set position of new filament within the specified radius around the gas source location
            Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
                return filament_spawn_distribution_(random_engine_); });

            Eigen::Vector3d filament_position = gas_source.position + vec_random;

            filaments_.emplace_back(filament_position,
                                    filament_initial_radius_,
                                    gas_source.mol_per_filament);
        }
    }
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
    (void)total_sim_time;

    Eigen::Vector3d new_position;

    // 1. Simulate Advection (Va)
    // Large scale wind-eddies -> Movement of a filament as a whole by wind
    // --------------------------------------------------------------------
    new_position = filament.position + time_step * wind_model_->getWindVelocityAt(filament.position);

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 2. Simulate Gravity & Bouyant Force
    // -----------------------------------
    // Stokes' law: Fd = 6 pi µ R v
    // Bouyancy: Fb = rho_air * 4/3 pi R^3
    // Gravity: Fg = rho_gas * 4/3 pi R^3
    // Equilibrium: Fd = Fb - Fg
    //     v = 2/9 (rho_air - rho_gas)/µ g R^2
    static double bouyancy_factor = 2.0/9.0 * constant::g
                                    / chemicals::Air::DynamicViscosity();
                                    // [m/s2] / [kg/(m*s)] = [m2/(kg*s)]

    static constexpr double fixed_R_squared = 1.0/4.0; // magic number from the original GADEN

    double terminal_buoyancy_velocity_z =
            bouyancy_factor * gas_density_factor_ * fixed_R_squared;
            // [m2/(kg*s)]  *       [kg/m3]       * [m2] = [m/s]

    Eigen::Vector3d terminal_buoyancy_velocity(
                0,
                0,
                terminal_buoyancy_velocity_z);

    new_position = filament.position + terminal_buoyancy_velocity * time_step;

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 3. Add some variability (stochastic process)
    // Vm (middle scale wind)-> Movement of the filament with respect to the
    // center of the "plume" -> modeled as Gaussian white noise
    // ---------------------------------------------------------------------
    Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
        return filament_stochastic_movement_distribution_(random_engine_); });

    new_position = filament.position + vec_random;

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 4. Filament growth with time (this affects the posterior estimation of
    //                               gas concentration at each cell)
    // Vd (small scale wind eddies) --> Difussion or change of the filament
    //                                  shape (growth with time)
    // R = sigma of a 3D gaussian --> Increasing sigma with time
    // ------------------------------------------------------------------------
    filament.addToSquaredRadius(filament_growth_gamma_ * time_step); // m2/s * s = m2

    return UpdatePositionResult::Okay;
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
