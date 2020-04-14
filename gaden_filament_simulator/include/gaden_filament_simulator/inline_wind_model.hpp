#ifndef GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED

#include "wind_model.hpp"

namespace gaden {

class InlineWindModel : public WindModel
{
    virtual void increment(double time_step, double total_sim_time);

    Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position);
};

} // namespace gaden

#endif // GADEN_SIMULATOR_INLINE_WIND_MODEL_HPP_INCLUDED
