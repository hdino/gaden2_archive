#ifndef GADEN_ENVIRONMENT_GRID_TO_MARKER_H_INCLUDED
#define GADEN_ENVIRONMENT_GRID_TO_MARKER_H_INCLUDED

#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gaden_common/occupancy_grid_type.h>

namespace gaden {

visualization_msgs::msg::MarkerArray
getAsMarkerArray(OccupancyGrid::Ptr grid,
                 const builtin_interfaces::msg::Time &stamp,
                 const std::string &frame_id,
                 rl::Logger &log);

} // namespace gaden

#endif // GADEN_ENVIRONMENT_GRID_TO_MARKER_H_INCLUDED
