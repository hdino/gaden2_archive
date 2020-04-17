#include <gaden_filament_simulator/wind_model.hpp>

namespace gaden {

WindModel::WindModel(rl::Logger &parent_logger)
    : logger(parent_logger.getChild("WindModel"))
{}

WindModel::~WindModel()
{
    logger.info("Destructing");
}

} // namespace gaden
