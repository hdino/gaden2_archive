#ifndef GADEN_SIMULATOR_WIND_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_WIND_MODEL_HPP_INCLUDED

#include <Eigen/Core>

namespace gaden {

class WindModel
{
public:
    virtual ~WindModel();

    virtual void increment(double time_step, double total_sim_time) = 0;

    virtual Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position) = 0;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_WIND_MODEL_HPP_INCLUDED
