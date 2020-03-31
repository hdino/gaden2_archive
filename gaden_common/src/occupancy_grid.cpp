#include <gaden_common/occupancy_grid.h>
#include <gaden_common/math_helper.h>

#include <openvdb/openvdb.h>

namespace gaden {

static void initialiseOpenVdb()
{
    static bool openvdb_initialised = false;

    if (!openvdb_initialised)
    {
        openvdb::initialize();
        openvdb_initialised = true;
    }
}

OccupancyGrid::Ptr createGrid()
{
    initialiseOpenVdb();

    static_assert(toUnderlying(Occupancy::Free) == 0,
                  "Occupancy grid background value should be 0.");
    return OccupancyGrid::create(); // if the background value is not 0, it must be specified here
}

OccupancyGrid::Ptr loadGridFromFile(const std::string &filename)
{
    initialiseOpenVdb();

    openvdb::io::File file(filename);
    file.open();

    for (openvdb::io::File::NameIterator name_it = file.beginName();
         name_it != file.endName(); ++name_it)
    {
        std::cout << name_it.gridName() << std::endl;
    }

    return OccupancyGrid::Ptr();
}

} // namespace gaden
