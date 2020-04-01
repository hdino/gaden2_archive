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

OccupancyGrid::Ptr createGrid(double cell_size)
{
    initialiseOpenVdb();

    static_assert(toUnderlying(Occupancy::Free) == 0,
                  "Occupancy grid background value should be 0.");

    // if the background value is not 0, it must be specified here
    OccupancyGrid::Ptr grid = OccupancyGrid::create();

    grid->insertMeta("cell_size", openvdb::DoubleMetadata(cell_size));

    return grid;
}

OccupancyGrid::Ptr loadGridFromFile(const std::string &filename,
                                    rl::Logger &log)
{
    initialiseOpenVdb();

    log.info() << "Loading occupancy grid file: " << filename;
    openvdb::io::File file(filename);
    file.open();

    unsigned grid_count = 0;
    std::string grid_name;
    for (openvdb::io::File::NameIterator name_it = file.beginName();
         name_it != file.endName(); ++name_it)
    {
        grid_name = name_it.gridName();
        ++grid_count;
    }

    if (grid_count == 0)
    {
        log.error() << filename << " does not contain any grid.";
        return OccupancyGrid::Ptr();
    }
    else if (grid_count > 1)
    {
        log.warn() << filename << " contains more than one grid."
                      " Will load grid with name: " << grid_name;
    }

    openvdb::GridBase::Ptr grid_base = file.readGrid(grid_name);
    file.close();

    return openvdb::gridPtrCast<OccupancyGrid>(grid_base);
}

} // namespace gaden
