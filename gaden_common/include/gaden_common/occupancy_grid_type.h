#ifndef GADEN_COMMON_OCCUPANCY_GRID_TYPE_H_INCLUDED
#define GADEN_COMMON_OCCUPANCY_GRID_TYPE_H_INCLUDED

#include <openvdb/openvdb.h>
//#include <openvdb/Grid.h>

namespace gaden {

//namespace open_vdb {
//using UInt8Tree = openvdb::tree::Tree4<uint8_t, 5, 4, 3>::Type;
//using UInt8Grid = openvdb::Grid<UInt8Tree>;
//}

using OccupancyGrid = openvdb::Int32Grid;//open_vdb::UInt8Grid;

enum class Occupancy : uint8_t { Free = 0, Occupied = 1, Outlet = 2 };

} // namespace gaden

#endif // GADEN_COMMON_OCCUPANCY_GRID_TYPE_H_INCLUDED
