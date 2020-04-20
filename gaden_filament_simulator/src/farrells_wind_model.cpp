// This is a C++ port of the Python code from the
// Insect Robotics Group, University of Edinburgh
// Available at: https://github.com/InsectRobotics/pompy

#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/farrells_wind_model.hpp>
#include <gaden_filament_simulator/farrells_wind_model_noise.hpp>

#include <yaml-cpp/yaml.h>

namespace gaden {

std::string toString(const FarrellsWindModelConfiguration &config, size_t indention)
{
    std::string newline = "\n" + std::string(indention, ' ');
    return             "u0: " + std::to_string(config.u0)
           + newline + "v0: " + std::to_string(config.v0)
           + newline + "Kx: " + std::to_string(config.k_x)
           + newline + "Ky: " + std::to_string(config.k_y)
           + newline + "Noise gain: " + std::to_string(config.noise_gain)
           + newline + "Noise damp: " + std::to_string(config.noise_damp)
           + newline + "Noise bandwidth: " + std::to_string(config.noise_bandwidth);
}

FarrellsWindModel::FarrellsWindModel(const YAML::Node &yaml_config,
                                     const std::shared_ptr<EnvironmentModel> &environment_model,
                                     rl::Logger &parent_logger)
    : WindModel(parent_logger)
    , environment_min_(environment_model->getEnvironmentMin())
{
    // load configuration
    YAML::Node wind_config = yaml_config["wind"]["farrell_wind"];

    double grid_target_size = wind_config["grid_target_size"].as<double>();

    config_.u0 = wind_config["u0"].as<double>();
    config_.v0 = wind_config["v0"].as<double>();
    config_.k_x = wind_config["k_x"].as<double>();
    config_.k_y = wind_config["k_y"].as<double>();
    config_.noise_gain = wind_config["noise_gain"].as<double>();
    config_.noise_damp = wind_config["noise_damp"].as<double>();
    config_.noise_bandwidth = wind_config["noise_bandwidth"].as<double>();

    logger.info() << "Farrell's wind model configuration:\n    " << toString(config_, 4);

    noise_generator_ = std::make_unique<FarrellColouredNoiseGenerator>(Eigen::Matrix2Xd::Zero(2, 8),
                                                                       config_.noise_damp,
                                                                       config_.noise_bandwidth,
                                                                       config_.noise_gain);

    // compute grid node spacing
    Eigen::Vector3d environment_size = environment_model->getEnvironmentMax() - environment_min_;
    logger.info() << "Environment size: x=" << environment_size[0] << " y=" << environment_size[1];
    logger.info() << "Grid target size: " << grid_target_size;

    // TODO
    size_t n_x = std::ceil(environment_size[0] / grid_target_size) + 1; // number of grid points in x direction
    size_t n_y = std::ceil(environment_size[1] / grid_target_size) + 1; // number of grid points in y direction

    dx_ = environment_size[0] / (n_x - 1);
    dy_ = environment_size[1] / (n_y - 1);
    dx_pow2_ = dx_ * dx_;
    dy_pow2_ = dy_ * dy_;
    delta_grid = Eigen::Vector3d(dx_, dy_, 1);

    logger.info() << "Using " << n_x << "x" << n_y << " grid points. --> dx=" << dx_ << " dy=" << dy_;

    // initialise wind velocity field to mean values
    // +2s are to account for boundary grid points
    //logger.info("Initialising wind velocity field");
    u_ = Eigen::ArrayXXd::Ones(n_x + 2, n_y + 2) * config_.u0;
    v_ = Eigen::ArrayXXd::Ones(n_x + 2, n_y + 2) * config_.v0;

    // preassign array of corner means values
    //logger.info("Initialising corner values");
    corner_means_.head<4>() = config_.u0;
    corner_means_.tail<4>() = config_.v0;
    //corner_means_.block(0, 0, 1, 4) = config_.u0;
    //corner_means_.block(0, 4, 1, 4) = config_.v0;
    //logger.info() << "Corner means: " << corner_means_;

    // precompute linear ramp arrays with size of boundary edges for
    // linear interpolation of corner values
    //logger.info("Initialising ramp arrays");
    ramp_x_ = Eigen::ArrayXd::LinSpaced(n_x + 2, 0.0, 1.0);
    ramp_y_ = Eigen::ArrayXd::LinSpaced(n_y + 2, 0.0, 1.0);
    //ramp_x_ = Eigen::VectorXd::LinSpaced(n_x + 2, 0.0, 1.0);
    //ramp_y_ = Eigen::VectorXd::LinSpaced(n_y + 2, 0.0, 1.0);

    increment(0.1, 0); // TODO repeat until steady state

    getWindVelocityAt(Eigen::Vector3d(-50, -25, 0));
    getWindVelocityAt(Eigen::Vector3d(-1, -1, 0));
    getWindVelocityAt(Eigen::Vector3d(0, 0, 0));
    getWindVelocityAt(Eigen::Vector3d(5, 5, 0));
    getWindVelocityAt(Eigen::Vector3d(50, 25, 0));
}

FarrellsWindModel::~FarrellsWindModel()
{}

void FarrellsWindModel::increment(double time_step, double total_sim_time)
{
    (void)total_sim_time;

    // update boundary values
    applyBoundaryConditions(time_step);
    //logger.info() << "u dim: " << u_.rows() << " " << u_.cols();

    // approximate spatial first derivatives with centred finite difference
    // equations for both components of wind field
    auto [du_dx, du_dy] = getCentred1stDifferences(u_);
    auto [dv_dx, dv_dy] = getCentred1stDifferences(v_);

    // approximate spatial second derivatives with centred finite difference
    // equations for both components of wind field
    auto [d2u_dx2, d2u_dy2] = getCentred2ndDifferences(u_);
    auto [d2v_dx2, d2v_dy2] = getCentred2ndDifferences(v_);

    // compute approximate time derivatives across simulation region
    // interior from defining PDEs
    // du/dt = -(u*du/dx + v*du/dy) + 0.5*k_x*d2u/dx2 + 0.5*k_y*d2u/dy2
    // dv/dt = -(u*dv/dx + v*dv/dy) + 0.5*k_x*d2v/dx2 + 0.5*k_y*d2v/dy2
    using namespace Eigen;
    auto u_inner = u_(seq(1, last-1), seq(1, last-1));
    auto v_inner = v_(seq(1, last-1), seq(1, last-1));

    logger.info() << u_inner.rows() << ", " << u_inner.cols();

    auto du_dt = -u_inner * du_dx - v_inner * du_dy +
                 0.5 * config_.k_x * d2u_dx2 + 0.5 * config_.k_y * d2u_dy2;
    auto dv_dt = -u_inner * dv_dx - v_inner * dv_dy +
                 0.5 * config_.k_x * d2v_dx2 + 0.5 * config_.k_y * d2v_dy2;

    // perform update with Euler integration
    u_inner += du_dt * time_step;
    v_inner += dv_dt * time_step;
}

void FarrellsWindModel::applyBoundaryConditions(double dt)
{
    noise_generator_->update(dt);

    // extract four corner values for each of u and v fields as component
    // mean plus current noise generator output
    Eigen::Array<double, 8, 1> uv_corners = noise_generator_->getNoise() + corner_means_;
    double u_tl = uv_corners[0]; double u_tr = uv_corners[1];
    double u_bl = uv_corners[2]; double u_br = uv_corners[3];
    double v_tl = uv_corners[4]; double v_tr = uv_corners[5];
    double v_bl = uv_corners[6]; double v_br = uv_corners[7];

    // linearly interpolate along edges
    using namespace Eigen;
    u_( all, 0   ) = u_tl + ramp_x_ * (u_tr - u_tl); // top edge
    u_( all, last) = u_bl + ramp_x_ * (u_br - u_bl); // bottom edge
    u_(   0, all ) = u_tl + ramp_y_ * (u_bl - u_tl); // left edge
    u_(last, all ) = u_tr + ramp_y_ * (u_br - u_tr); // right edge

    v_( all, 0   ) = v_tl + ramp_x_ * (v_tr - v_tl); // top edge
    v_( all, last) = v_bl + ramp_x_ * (v_br - v_bl); // bottom edge
    v_(   0, all ) = v_tl + ramp_y_ * (v_bl - v_tl); // left edge
    v_(last, all ) = v_tr + ramp_y_ * (v_br - v_tr); // right edge
}

std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> FarrellsWindModel::getCentred1stDifferences(const Eigen::ArrayXXd &f)
{
    using namespace Eigen;
    return {(f(seq(2, last),   seq(1, last-1)) - f(seq(0, last-2), seq(1, last-1))) / (2 * dx_), // TODO Precompute 1/(2dx)
            (f(seq(1, last-1), seq(2, last))   - f(seq(1, last-1), seq(0, last-2))) / (2 * dy_)};
}

std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> FarrellsWindModel::getCentred2ndDifferences(const Eigen::ArrayXXd &f)
{
    using namespace Eigen;
    return {(f(seq(2, last),   seq(1, last-1)) - 2 * f(seq(1, last-1), seq(1, last-1)) + f(seq(0, last-2), seq(1, last-1))) / dx_pow2_, // TODO Precompute
            (f(seq(1, last-1), seq(2, last))   - 2 * f(seq(1, last-1), seq(1, last-1)) + f(seq(1, last-1), seq(0, last-2))) / dy_pow2_};
}

Eigen::Vector3d FarrellsWindModel::getWindVelocityAt(const Eigen::Vector3d &position)
{
    using namespace Eigen;
    auto u_inner = u_(seq(1, last-1), seq(1, last-1));
    auto v_inner = v_(seq(1, last-1), seq(1, last-1));

    Eigen::Array3d cell_float_position = (position - environment_min_).array() / delta_grid.array();
    Eigen::Array3d cell_lower_index_float = cell_float_position.floor();
    Eigen::Array3d cell_position_offset = cell_float_position - cell_lower_index_float;
    logger.info() << "Offset\n" << cell_position_offset;

    Eigen::Array3i cell_lower_index = cell_lower_index_float.cast<int>();
    Eigen::Array3i cell_upper_index = cell_lower_index + 1;

    double u = (1 - cell_position_offset[0]) * u_(cell_lower_index[0], cell_lower_index[1])
                  + cell_position_offset[0]  * u_(cell_upper_index[0], cell_upper_index[1]);

    double v = (1 - cell_position_offset[1]) * v_(cell_lower_index[0], cell_lower_index[1])
                  + cell_position_offset[1]  * v_(cell_upper_index[0], cell_upper_index[1]);

    return Eigen::Vector3d(1,0,0);
}

} // namespace gaden
