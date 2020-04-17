#ifndef GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED

#include <memory>
#include <random>

#include <Eigen/Core>
#include <openvdb/openvdb.h>

#include "wind_model.hpp"

namespace YAML {
class Node;
}

namespace gaden {

template <typename TGrid>
class CacheGrid;

class InlineWindModel : public WindModel
{
public:
    InlineWindModel(const YAML::Node &config, rl::Logger &parent_logger);
    ~InlineWindModel();

    virtual void increment(double time_step, double total_sim_time);

    Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position);

private:
    openvdb::Vec3d generateWindVelocityAt(const Eigen::Vector3d &position);

    std::unique_ptr<CacheGrid<openvdb::Vec3DGrid>> cache_;

    double t_next_update_; // [s]
    double time_between_updates_; // [s]

    std::default_random_engine random_engine_;
    std::normal_distribution<double> random_distribution_u_;
    std::normal_distribution<double> random_distribution_v_;
    std::normal_distribution<double> random_distribution_w_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED
