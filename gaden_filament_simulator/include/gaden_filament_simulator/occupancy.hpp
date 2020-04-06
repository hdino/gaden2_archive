#ifndef GADEN_SIMULATOR_OCCUPANCY_HPP_INCLUDED
#define GADEN_SIMULATOR_OCCUPANCY_HPP_INCLUDED

#include <rl_logging/logging_interface.hpp>

namespace gaden {

struct SimulatorConfig;

bool generateOccupancyFile(const SimulatorConfig &config, rl::Logger &logger);

} // namespace gaden

#endif // GADEN_SIMULATOR_OCCUPANCY_HPP_INCLUDED
