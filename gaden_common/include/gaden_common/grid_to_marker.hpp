#ifndef GADEN_COMMON_GRID_TO_MARKER_HPP_INCLUDED
#define GADEN_COMMON_GRID_TO_MARKER_HPP_INCLUDED

#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "occupancy_grid_type.h"

namespace gaden {

visualization_msgs::msg::MarkerArray
getAsMarkerArray(OccupancyGrid::Ptr grid,
                 const builtin_interfaces::msg::Time &stamp,
                 const std::string &frame_id,
                 rl::Logger &log);

} // namespace gaden

#endif // GADEN_COMMON_GRID_TO_MARKER_HPP_INCLUDED
