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

FilamentModel::FilamentModel(
        const YAML::Node &config,
        std::shared_ptr<EnvironmentModel> environment_model,
        rl::Logger &parent_logger)
    : GasDispersionModel(parent_logger)
    , random_engine_(std::random_device()())
    , env_model_(environment_model)
    , filament_grid_(1.0, env_model_)
{
    // load configuration parameters
    YAML::Node model_config = config["filament_model"];

    double filament_spawn_radius = 1.0; // [m]
        // radius around the source in which the filaments are spawned

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

    gas_density_delta_ = chemicals::Air::MassDensity() - gas_->getMassDensity(); // [kg/m3]
    // TODO Make the mass density depend on temperature (assuming pressure is always 1 atm?)

    logger.info() << "Gas dispersion model: Filament model";
}

void FilamentModel::processSimulatorSet()
{
    wind_model_ = simulator->getWindModel();

    // Compute filament release rate of each gas source
    for (const GasSource &gas_source : simulator->getGasSources())
    {
        FilamentGasSource filament_gas_source;
        static_cast<GasSource &>(filament_gas_source) = gas_source;

        double molar_release_rate = gas_source.release_rate // [kg/h] /
                                    / gas_->getMolarMass()  // [kg/mol] /
                                    / 3600.0;               // [s/h] = [mol/s]

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

/**
 * @brief FilamentModel::getMolarFraction
 * @param gas_molar_concentration in [mol/m3]
 * @return The amount of the gas divided by the total amount of all
 *         constituents in an ideal gas (dimensionless, []).
 */
double FilamentModel::getMolarFraction(double gas_molar_concentration) const
{
    static constexpr double IDEAL_GAS_M3_PER_MOL =
            constant::R *                            // [m3 Pa / (K * mol)] *
            constant::toKelvinFromDegreeCelsius(0) / // [K] /
            constant::toPascalFromAtmosphere(1);     // [Pa] = [m3/mol]

    static constexpr double IDEAL_GAS_MOLAR_CONCENTRATION =
            1.0 / IDEAL_GAS_M3_PER_MOL;              // [mol/m3]

    return gas_molar_concentration / IDEAL_GAS_MOLAR_CONCENTRATION; // []
}

double FilamentModel::getConcentrationAt(const Eigen::Vector3d &position)
{
    double concentration = 0; // [mol/m3]

//    const std::vector<FilamentInfluence> &neighbour_filaments =
//            filament_grid_.getFilamentsAt(position);

//    for (const FilamentInfluence &item : neighbour_filaments)
//    {
//        if (item.need_to_check_for_obstacle &&
//                env_model_->hasObstacleBetweenPoints(position, item.filament->position))
//            continue;
//        concentration += item.filament->getConcentrationAt(position);
//    }

    for (const Filament &filament : filaments_)
    {
//        if (env_model_->hasObstacleBetweenPoints(position, filament.position))
//            continue;
        concentration += filament.getConcentrationAt(position);
    }

    // convert mol/m3 to ppm
    return getMolarFraction(concentration) * 1e6;
}

void FilamentModel::increment(double time_step, double total_sim_time)
{
    addNewFilaments(time_step, total_sim_time);
    updateFilamentPositions(time_step, total_sim_time);

//    filament_grid_.clear();
//    for (Filament &filament : filaments_)
//        filament_grid_.add(&filament);

//    static double max_r2 = 0;
//    for (const Filament &filament : filaments_)
//        if (filament.getSquaredRadius() > max_r2)
//            max_r2 = filament.getSquaredRadius();

//    logger.info() << "#filaments: " << filaments_.size() << " max_r2=" << max_r2;
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
    case Occupancy::OutOfWorld:
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
    static double bouyancy_factor =
            2.0/9.0 *                           // [] *
            constant::g /                       // [m/s2] /
            chemicals::Air::DynamicViscosity(); // [kg/(m*s)] = [m2/(kg*s)]

    //double radius_squared = filament.getSquaredRadius() * 0.75e-2; // [m2]
    double radius_squared = filament.getSquaredRadius() * 0.05; // [m2]
        // TODO: magic factor (0.75e-2), there was one in the original GADEN as well
    //double gas_fraction = getMolarFraction(filament.getConcentrationAtCentre()); // []
    double radius = 3.0 * std::sqrt(filament.getSquaredRadius());
    double radius_pow3 = radius*radius*radius;
    double average_concentration = filament.gas_amount / (4/3 * M_PI * radius_pow3);
    double gas_fraction = getMolarFraction(average_concentration);

    double terminal_buoyancy_velocity_z =
            bouyancy_factor *       // [m2/(kg*s)] *
            gas_density_delta_ *    // [kg/m3] *
            radius_squared *        // [m2] *
            gas_fraction;           // [] = [m/s]

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
    double radius_squared_delta = filament_growth_gamma_ * time_step; // m2/s * s = m2
    filament.addToSquaredRadius(radius_squared_delta);

    return UpdatePositionResult::Okay;
}

} // namespace gaden
