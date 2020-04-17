#include <gaden_common/cache_grid.hpp>
#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/inline_wind_model.hpp>

#include <yaml-cpp/yaml.h>

#include <limits>

namespace gaden {

InlineWindModel::InlineWindModel(const YAML::Node &config, rl::Logger &parent_logger)
    : WindModel(parent_logger)
    , t_next_update_(0)
    , random_engine_(std::random_device()())
{
    // load configuration
    YAML::Node wind_config = config["wind"]["inline_wind"];

    double cell_size = wind_config["cell_size"].as<double>(); // [m]
    time_between_updates_ = wind_config["time_between_updates"].as<double>(); // [s]

    Eigen::Vector3d wind_speed = eigen_helper::getVector3dFromYaml(wind_config["speed"]);
    Eigen::Vector3d wind_speed_std_dev = eigen_helper::getVector3dFromYaml(wind_config["speed_std_dev"]);

    logger.info() << "Inline wind model"
                  << "\n  Cell size: " << cell_size
                  << "\n  Speed: " << toString(wind_speed)
                  << "\n  Std dev: " << toString(wind_speed_std_dev)
                  << "\n  Update time: " << time_between_updates_;

    // set up random distributions
    random_distribution_u_ = std::normal_distribution<double>(wind_speed[0], wind_speed_std_dev[0]);
    random_distribution_v_ = std::normal_distribution<double>(wind_speed[1], wind_speed_std_dev[1]);
    random_distribution_w_ = std::normal_distribution<double>(wind_speed[2], wind_speed_std_dev[2]);

    cache_ = std::make_unique<CacheGrid<openvdb::Vec3DGrid>>(
                cell_size,
                [this](auto&& ...x) {
                    return generateWindVelocityAt(std::forward<decltype(x)>(x)...); });
}

InlineWindModel::~InlineWindModel()
{}

void InlineWindModel::increment(double time_step, double total_sim_time)
{
    (void)time_step;

    if (total_sim_time >= t_next_update_)
    {
        cache_->clear();
        t_next_update_ = total_sim_time + time_between_updates_;
    }
}

Eigen::Vector3d InlineWindModel::getWindVelocityAt(const Eigen::Vector3d &position)
{
    return openvdb_helper::toEigen(cache_->getValue(position));
}

openvdb::Vec3d InlineWindModel::generateWindVelocityAt(const Eigen::Vector3d &position)
{
    (void)position;
    return openvdb::Vec3d(random_distribution_u_(random_engine_),
                          random_distribution_v_(random_engine_),
                          random_distribution_w_(random_engine_));
}

} // namespace gaden
