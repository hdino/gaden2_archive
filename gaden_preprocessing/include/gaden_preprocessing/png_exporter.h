#ifndef GADEN_PREPROCESSING_PNG_EXPORTER_H_INCLUDED
#define GADEN_PREPROCESSING_PNG_EXPORTER_H_INCLUDED

#include <string>

#include <rclcpp/logger.hpp>

#include <gaden_common/occupancy_grid_type.h>

namespace gaden {

/** exportPng: Save a 2D top view of grid into png_file.
 * Each cell is represented by scale * scale pixels.
 * Colour legend:
 *     white: free space
 *     black: ground cell is occupied (i.e. where z = environment_min),
 *            cells with z > environment_min might also be occupied
 *     grey:  at least one cell above ground is occupied (ground cell is not)
 *     red:   outlet (not distinguished between ground cell and others)
**/
void exportPng(OccupancyGrid::Ptr &grid, const std::string &png_file,
               rclcpp::Logger &logger, unsigned scale = 1);

} // namespace gaden

#endif // GADEN_PREPROCESSING_PNG_EXPORTER_H_INCLUDED
