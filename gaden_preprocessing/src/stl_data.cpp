#include <gaden_preprocessing/stl_data.h>
#include <gaden_preprocessing/stl_to_grid.h>

namespace gaden {

StlData::StlData(std::vector<StlFacet> &&facets)
    : facets_(std::move(facets))
{
    //
}

bool StlData::isEmpty() const
{
    return facets_.empty();
}

void StlData::addToOccupancyGrid(OccupancyGrid::Ptr &grid,
                                 Occupancy occupancy_type,
                                 double cell_size) const
{
    addStlToGrid(facets_, grid, occupancy_type, cell_size);
}

} // namespace gaden

