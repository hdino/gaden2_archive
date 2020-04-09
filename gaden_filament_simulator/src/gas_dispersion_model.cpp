#include <gaden_filament_simulator/gas_dispersion_model.hpp>
//#include <gaden_filament_simulator/simulator.hpp>

namespace gaden {

GasDispersionModel::GasDispersionModel(double cell_size, rl::Logger &parent_logger)
    : logger(parent_logger.getChild("GasDispersionModel"))
    , cell_size(cell_size)
{
    logger.info() << "Cell size: " << cell_size;
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
