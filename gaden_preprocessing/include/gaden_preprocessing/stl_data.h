#ifndef GADEN_PREPROCESSING_STL_DATA_H_INCLUDED
#define GADEN_PREPROCESSING_STL_DATA_H_INCLUDED

#include <array>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>
#include <Eigen/Core>

#include "occupancy_grid_type.h"

namespace gaden {

struct StlFacet
{
    //Eigen::Vector3f facet_normal;
    //Eigen::Vector3f vertex[3];
    openvdb::Vec3s facet_normal;
    std::array<openvdb::Vec3s, 3> vertices;
};

class StlData : private boost::noncopyable
{
public:
    StlData(std::vector<StlFacet> &&facets);

    bool isEmpty() const;

    void addToOccupancyGrid(OccupancyGrid::Ptr &grid,
                            Occupancy occupancy_type,
                            double cell_size) const;

private:
    std::vector<StlFacet> facets_;
};

} // namespace gaden

#endif // GADEN_PREPROCESSING_STL_DATA_H_INCLUDED
