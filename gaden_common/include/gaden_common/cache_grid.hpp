#ifndef GADEN_COMMON_CACHE_GRID_HPP_INCLUDED
#define GADEN_COMMON_CACHE_GRID_HPP_INCLUDED

#include <functional>

#include <Eigen/Core>
#include <openvdb/openvdb.h>

#include "openvdb_coordinates.hpp"
#include "openvdb_helper.h"

namespace gaden {

template <typename TGrid>
class CacheGrid
{
public:
    using GeneratorFunction = std::function<typename TGrid::ValueType (const Eigen::Vector3d &)>;

    CacheGrid(double cell_size, GeneratorFunction generator)
        : coord_tf_(cell_size)
        , generateValue_(generator)
        , openvdb_initialised_(initialiseOpenVdb())
        , grid_(TGrid::create())
        , grid_accessor_(grid_->getAccessor())
    {}

    typename TGrid::ValueType getValue(const Eigen::Vector3d &p)
    {
        typename TGrid::ValueType result;

        openvdb::Coord coord = coord_tf_.toCellCoordinates(p);
        if (!grid_accessor_.probeValue(coord, result)) // not cached yet
        {
            // generate the value at the centre of the cell, not at p
            result = generateValue_(coord_tf_.toWorldCoordinatesCenter(coord));
            grid_accessor_.setValue(coord, result);
        }

        return result;
    }

    void clear()
    {
        grid_->clear();
    }

private:
    open_vdb::CoordinateTransformer coord_tf_;
    GeneratorFunction generateValue_;

    bool openvdb_initialised_; // dummy variable to make sure OpenVDB is initialised
    typename TGrid::Ptr grid_;
    typename TGrid::Accessor grid_accessor_;
};

} // namespace gaden

#endif // GADEN_COMMON_CACHE_GRID_HPP_INCLUDED
