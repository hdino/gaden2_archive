#ifndef GADEN_SIMULATOR_GAS_DISPERSION_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_GAS_DISPERSION_MODEL_HPP_INCLUDED

#include <memory>

#include <rl_logging/logging_interface.hpp>

namespace gaden {

class Simulator;

class GasDispersionModel
{
public:
    GasDispersionModel(rl::Logger &parent_logger);
    virtual ~GasDispersionModel();

    virtual void increment(double time_step, double total_sim_time) = 0;

    void setSimulator(std::shared_ptr<Simulator> simulator);

protected:
    rl::Logger logger;

    std::shared_ptr<Simulator> simulator;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_GAS_DISPERSION_MODEL_HPP_INCLUDED
