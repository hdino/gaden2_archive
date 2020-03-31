#include <iostream>

#include <gaden_common/gaden1_occupancy_grid_importer.h>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Wrong number of parameters.\nUsage: " << argv[0]
                  << " OccupancyGrid3D.csv OccupancyGrid.vdb" << std::endl;
        return 0;
    }

    gaden::OccupancyGrid::Ptr grid = gaden::importOccupancyGridFromGaden1(argv[1]);
    if (grid->empty())
    {
        std::cout << "Importing occupancy grid from " << argv[1] << " failed." << std::endl;
        return 0;
    }

    return 0;
}
