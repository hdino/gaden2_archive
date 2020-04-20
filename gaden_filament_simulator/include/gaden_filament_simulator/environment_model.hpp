#ifndef GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

#include <gaden_common/occupancy.hpp>

namespace gaden {

class EnvironmentModel
{
public:
    EnvironmentModel(rl::Logger &parent_logger);
    virtual ~EnvironmentModel();

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

    virtual bool hasObstacleBetweenPoints(const Eigen::Vector3d &pa,
                                          const Eigen::Vector3d &pb) const = 0;

    virtual Occupancy getOccupancy(const Eigen::Vector3d &p) const = 0;

protected:
    rl::Logger logger;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED
