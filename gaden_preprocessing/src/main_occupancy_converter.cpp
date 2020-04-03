#include <iostream>

#include <rl_logging/std_logging.hpp>

#include <gaden_common/gaden1_occupancy_grid_importer.h>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Wrong number of parameters.\nUsage: " << argv[0]
                  << " OccupancyGrid3D.csv OccupancyGrid.vdb" << std::endl;
        return 0;
    }

    rl::Logger logger = rl::logging::StdLogger::create("GadenOccupancyConverter");

    gaden::OccupancyGrid::Ptr grid = gaden::importOccupancyGridFromGaden1(argv[1], logger);
    if (!grid || grid->empty())
    {
        logger.error() << "Importing occupancy grid from " << argv[1] << " failed.";
        return 0;
    }

    std::string occupancy_grid_file = argv[2];
    logger.info() << "Writing occupancy grid to: " << occupancy_grid_file;
    openvdb::io::File(occupancy_grid_file).write({grid});

    return 0;
}
