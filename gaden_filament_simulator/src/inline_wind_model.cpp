#include <gaden_filament_simulator/inline_wind_model.hpp>

namespace gaden {

void InlineWindModel::increment(double time_step, double total_sim_time)
{
    //
}

Eigen::Vector3d InlineWindModel::getWindVelocityAt(const Eigen::Vector3d &position)
{
    return Eigen::Vector3d(1, 0, 0);
}

} // namespace gaden
