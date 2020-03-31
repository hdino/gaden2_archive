#include <gaden_common/gaden1_occupancy_grid_importer.h>
#include <gaden_common/occupancy_grid.h>

#include <array>
#include <fstream>
#include <iostream>

namespace gaden {

OccupancyGrid::Ptr importOccupancyGridFromGaden1(const std::string &filename)
{
    OccupancyGrid::Ptr grid = createGrid();

    std::ifstream file_stream(filename.c_str());
    if (!file_stream.is_open())
    {
        std::cout << "Failed to open " << filename
                  << " : " << strerror(errno) << std::endl;
        return grid;
    }

    // 1st step: Read the header of the OccupancyGrid3D.csv file.
    // Its header looks like (# is the first character of each line):
    //     #env_min(m) -5.0000 -5.5000 0.0000
    //     #env_max(m) 5.0000 5.5000 3.0000
    //     #num_cells 100 110 30
    //     #cell_size(m) 0.1000

    std::array<std::string, 4> header_lines;
    size_t i;
    for (i = 0;
         i < header_lines.size() && std::getline(file_stream, header_lines[i]);
         ++i);

    if (i != header_lines.size())
    {
        if (file_stream.bad())
            std::cout << "Error while reading " << filename
                      << " : " << strerror(errno) << std::endl;
        else
            std::cout << "Error while reading header of "
                      << filename << std::endl;
        return grid;
    }

    openvdb::Vec3d env_min, env_max;
    openvdb::math::Vec3<size_t> num_cells;

//    while (std::getline(file_stream, line))
//    {
//        if (line.rfind("solid", 0) == 0)
//        {
//            found_solid = true;
//            break;
//        }
//    }

    return grid;
}

} // namespace gaden
