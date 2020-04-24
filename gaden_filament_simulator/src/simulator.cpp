#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/gas_dispersion_model.hpp>
#include <gaden_filament_simulator/simulator.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>
#include <gaden_filament_simulator/wind_model.hpp>

#include <thread>

namespace gaden {

struct Simulator::MakeConstructorPublic : public Simulator {
    template <typename... Args>
    MakeConstructorPublic(Args&& ...args)
        : Simulator(std::forward<Args>(args)...)
    {}
};

std::shared_ptr<Simulator> Simulator::create(SimulatorConfig &config,
                                             std::shared_ptr<EnvironmentModel> env_model,
                                             std::shared_ptr<GasDispersionModel> gas_model,
                                             std::shared_ptr<WindModel> wind_model,
                                             rl::Logger &logger)
{
    std::shared_ptr<Simulator> sim =
            std::make_shared<MakeConstructorPublic>(config,
                                                    env_model,
                                                    gas_model,
                                                    wind_model,
                                                    logger);
    sim->init();
    return sim;
}

Simulator::Simulator(SimulatorConfig &config,
                     std::shared_ptr<EnvironmentModel> env_model,
                     std::shared_ptr<GasDispersionModel> gas_model,
                     std::shared_ptr<WindModel> wind_model,
                     rl::Logger &logger)
    : logger_(logger.getChild("Simulator"))
    , environment_(env_model)
    , gas_model_(gas_model)
    , wind_model_(wind_model)
    , gas_sources_(config.gas_sources)
{
    // load configuration parameters
    time_step_ = 0.2;
    double end_time = 120.0;

    // initialise internal state
    current_sim_step_ = 0;
    total_sim_steps_ = std::ceil(end_time / time_step_);

    logger_.info() << "end_time = " << end_time << " s   time_step = " << time_step_ << " s   sim_steps = " << total_sim_steps_;
    logger_.info() << "Number of gas sources: " << gas_sources_.size();
}

void Simulator::init()
{
    logger_.info("Registering simulator with the gas dispersion model...");
    gas_model_.lock()->setSimulator(shared_from_this());
    logger_.info("Registered.");
}

Simulator::~Simulator()
{
    logger_.info("Destructing");
}

bool Simulator::simulate()
{
    if (current_sim_step_ > total_sim_steps_)
        return false;

    double sim_time = current_sim_step_ * time_step_;

    if (current_sim_step_ % 10 == 0)
        logger_.info() << "Simulating... sim_time = " << sim_time;

    wind_model_.lock()->increment(time_step_, sim_time);
    gas_model_.lock()->increment(time_step_, sim_time);

    //    if ( (sim.save_results==1) && (sim.sim_time>=sim.results_min_time) )
    //    {
    //        if ( floor(sim.sim_time/sim.results_time_step) != sim.last_saved_step )
    //            sim.save_state_to_file();
    //    }

    //sim_time_ += time_step_;
    ++current_sim_step_;

    std::this_thread::sleep_for(std::chrono::milliseconds(unsigned(time_step_ * 250)));

    return true;
}

} // namespace gaden
