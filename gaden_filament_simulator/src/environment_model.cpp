#include <gaden_filament_simulator/environment_model.hpp>

namespace gaden {

EnvironmentModel::EnvironmentModel(rl::Logger &parent_logger)
    : logger(parent_logger.getChild("EnvironmentModel"))
{}

EnvironmentModel::~EnvironmentModel()
{
    logger.info("Destructing...");
}

} // namespace gaden
