#ifndef GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden {

class EnvironmentModel
{
public:
    EnvironmentModel(rl::Logger &parent_logger);
    virtual ~EnvironmentModel();

    virtual bool hasObstacleBetweenPoints(const Eigen::Vector3d &pa,
                                          const Eigen::Vector3d &pb) const = 0;

protected:
    rl::Logger logger;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_ENVIRONMENT_MODEL_HPP_INCLUDED
