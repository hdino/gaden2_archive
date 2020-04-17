#ifndef GADEN_COMMON_OPENVDB_COORDINATES_HPP_INCLUDED
#define GADEN_COMMON_OPENVDB_COORDINATES_HPP_INCLUDED

#include <Eigen/Core>
#include <openvdb/Types.h>

#include "grid_helper.hpp"

namespace gaden::open_vdb {

class CoordinateTransformer
{
public:
    inline CoordinateTransformer(double cell_size)
        : cell_size_(cell_size)
    {}

    inline double getCellSize() const { return cell_size_; }

    inline openvdb::Coord toCellCoordinates(Eigen::Vector3d v) const
    {
        return grid_helper::getCellCoordinates(v, cell_size_);
    }

    inline Eigen::Vector3d toWorldCoordinatesOrigin(const openvdb::Coord &coord) const
    {
        Eigen::Vector3d v(coord.x(), coord.y(), coord.z());
        return cell_size_ * v;
    }

    inline Eigen::Vector3d toWorldCoordinatesCenter(const openvdb::Coord &coord) const
    {
        return toWorldCoordinatesOrigin(coord).array() + 0.5 * cell_size_;
    }

private:
    double cell_size_;
};

} // namespace gaden::open_vdb

#endif // GADEN_COMMON_OPENVDB_COORDINATES_HPP_INCLUDED
