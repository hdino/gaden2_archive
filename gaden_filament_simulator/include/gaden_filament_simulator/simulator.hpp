#ifndef GADEN_SIMULATOR_SIMULATOR_HPP_INCLUDED
#define GADEN_SIMULATOR_SIMULATOR_HPP_INCLUDED

#include <memory>
#include <vector>

#include <rl_logging/logging_interface.hpp>

#include <gaden_common/gas_source.hpp>

namespace gaden {

struct SimulatorConfig;

class EnvironmentModel;
class GasDispersionModel;
class WindModel;

class Simulator : public std::enable_shared_from_this<Simulator>
{
public:
    static std::shared_ptr<Simulator> create(SimulatorConfig &config,
                                             std::shared_ptr<EnvironmentModel> env_model,
                                             std::shared_ptr<GasDispersionModel> gas_model,
                                             std::shared_ptr<WindModel> wind_model,
                                             rl::Logger &logger);

    ~Simulator();

    bool simulate(); // returns false when simulation has finished, true otherwise

    std::shared_ptr<EnvironmentModel> getEnvironmentModel() { return environment_.lock(); }
    std::shared_ptr<GasDispersionModel> getGasModel() { return gas_model_.lock(); }
    std::shared_ptr<WindModel> getWindModel() { return wind_model_.lock(); }

    const std::vector<GasSource> & getGasSources() const { return gas_sources_; }

    //unsigned getTotalSteps() const { return total_sim_steps_; }

private:
    struct MakeConstructorPublic;

    Simulator(SimulatorConfig &config,
              std::shared_ptr<EnvironmentModel> env_model,
              std::shared_ptr<GasDispersionModel> gas_model,
              std::shared_ptr<WindModel> wind_model,
              rl::Logger &logger);

    void init();

    rl::Logger logger_;

    std::weak_ptr<EnvironmentModel> environment_;
    std::weak_ptr<GasDispersionModel> gas_model_;
    std::weak_ptr<WindModel> wind_model_;

    // internal state parameters
    unsigned total_sim_steps_;
    unsigned current_sim_step_;
    //double sim_time_; // [s] current time of the simulation

    // configuration parameters
    double time_step_; // [s] time increment between snapshots

    std::vector<GasSource> gas_sources_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_SIMULATOR_HPP_INCLUDED
