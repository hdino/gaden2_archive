#ifndef GADEN_PREPROCESSING_STL_TO_GRID_H_INCLUDED
#define GADEN_PREPROCESSING_STL_TO_GRID_H_INCLUDED

#include <vector>

#include "occupancy_grid.h"
#include "stl_data.h"

namespace gaden {

using Vec3Bool = openvdb::math::Vec3<bool>;

void addStlToGrid(const std::vector<StlFacet> &facets,
                  OccupancyGrid::Ptr &grid,
                  double cell_size);

void processVertex(OccupancyGrid::Accessor &grid_accessor,
                   const openvdb::Vec3i &vertex_cell_coordinates,
                   const openvdb::Vec3i &max_vertex_cell_coordinates,
                   const Vec3Bool &limit);

bool isParallelToBasis(const openvdb::Vec3s &vector);

bool isPointInTriangle(const Eigen::Vector3d& query_point,
                       const Eigen::Vector3d& triangle_vertex_0,
                       const Eigen::Vector3d& triangle_vertex_1,
                       const Eigen::Vector3d& triangle_vertex_2,
                       bool parallel, double cell_size);

std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point,
                                        double cell_size);

bool planeIntersects(const Eigen::Vector3d &n,
                     const Eigen::Vector3d &plane_point,
                     const std::vector<Eigen::Vector3d> &cube);

} // namespace gaden

#endif // GADEN_PREPROCESSING_STL_TO_GRID_H_INCLUDED
