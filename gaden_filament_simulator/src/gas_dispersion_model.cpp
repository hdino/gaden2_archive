#include <gaden_filament_simulator/gas_dispersion_model.hpp>
//#include <gaden_filament_simulator/simulator.hpp>

namespace gaden {

GasDispersionModel::GasDispersionModel(rl::Logger &parent_logger)
    : logger(parent_logger.getChild("GasDispersionModel"))
{
    //
}

GasDispersionModel::~GasDispersionModel()
{
    logger.info("Destructing");
}

void GasDispersionModel::setSimulator(std::shared_ptr<Simulator> simulator)
{
    this->simulator = simulator;
}

} // namespace gaden
