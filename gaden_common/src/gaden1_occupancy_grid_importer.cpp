#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/file_read_helper.h>
#include <gaden_common/gaden1_occupancy_grid_importer.h>
#include <gaden_common/grid_helper.hpp>
#include <gaden_common/occupancy_grid.h>
#include <gaden_common/openvdb_helper.h>

#include <array>
#include <charconv>
#include <fstream>
#include <sstream>
#include <system_error>

#include <Eigen/Core>

namespace gaden {

OccupancyGrid::Ptr importOccupancyGridFromGaden1(const std::string &filename,
                                                 rl::Logger &logger)
{
    OccupancyGrid::Ptr grid;

    std::ifstream file_stream(filename.c_str());
    if (!file_stream.is_open())
    {
        logger.error() << "Failed to open " << filename
                       << " : " << strerror(errno);
        return grid;
    }
    logger.info() << "Opened " << filename;

    // 1st step: Read the header of the OccupancyGrid3D.csv file.
    // Its header looks like (# is the first character of each line):
    //     #env_min(m) -5.0000 -5.5000 0.0000
    //     #env_max(m) 5.0000 5.5000 3.0000
    //     #num_cells 100 110 30
    //     #cell_size(m) 0.1000
    std::stringstream stream_line;

    //openvdb::Vec3d env_min, env_max;
    Eigen::Vector3d env_min, env_max;
    openvdb::math::Vec3<size_t> num_cells;
    double cell_size;

    if (!file_read_helper::getLineAndSkipStart(
                file_stream, "#env_min(m) ", stream_line, logger)) return grid;
    if (!eigen_helper::getFromStream(stream_line, env_min))
    {
        logger.error() << "env_min line invalid in " << filename;
        return grid;
    }

    if (!file_read_helper::getLineAndSkipStart(
                file_stream, "#env_max(m) ", stream_line, logger)) return grid;
    if (!eigen_helper::getFromStream(stream_line, env_max))
    {
        logger.error() << "env_max line invalid in " << filename;
        return grid;
    }

    if (!file_read_helper::getLineAndSkipStart(
                file_stream, "#num_cells ", stream_line, logger)) return grid;
    if (!openvdb_helper::getFromStream(stream_line, num_cells))
    {
        logger.error() << "num_cells line invalid in " << filename;
        return grid;
    }

    if (!file_read_helper::getLineAndSkipStart(
                file_stream, "#cell_size(m) ", stream_line, logger)) return grid;
    if (!static_cast<bool>(stream_line >> cell_size))
    {
        logger.error() << "cell_size line invalid in " << filename;
        return grid;
    }

    logger.info() << "Finished reading metadata:"
                  << "\n    env_min:   " << toString(env_min)
                  << "\n    env_max:   " << toString(env_max)
                  << "\n    num_cells: " << num_cells.str()
                  << "\n    cell_size: " << cell_size;

    grid = createGrid(cell_size);
    auto grid_accessor = grid->getAccessor();


    //openvdb::Vec3i cell_index_min = getCellCoordinates(env_min, cell_size);
    //openvdb::Vec3i cell_index_max = getCellCoordinates(env_max, cell_size);
    openvdb::Coord cell_index_min = grid_helper::getCellCoordinates(env_min, cell_size);
    openvdb::Coord cell_index_max = grid_helper::getCellCoordinates(env_max, cell_size);
    logger.info() << "Minimum cell index: " << toString(cell_index_min);
    logger.info() << "Maximum cell index: " << toString(cell_index_max);

//    std::array<std::string, 4> header_lines;
//    size_t i;
//    for (i = 0;
//         i < header_lines.size() && std::getline(file_stream, header_lines[i]);
//         ++i);

//    if (i != header_lines.size())
//    {
//        if (file_stream.bad())
//            std::cout << "Error while reading " << filename
//                      << " : " << strerror(errno) << std::endl;
//        else
//            std::cout << "Error while reading header of "
//                      << filename << std::endl;
//        return grid;
//    }

    openvdb::Coord current_cell(cell_index_min);
    std::string line;
    std::from_chars_result conversion_result;
    int cell_value;
    while (std::getline(file_stream, line))
    {
        if (line == ";")
        {
            ++current_cell.z();
            current_cell.x() = cell_index_min.x();
            continue;
        }

        conversion_result.ptr = line.data();
        const char *end_ptr = line.data() + line.size();

        for (current_cell.y() = cell_index_min.y();
             current_cell.y() < cell_index_max.y();
             ++current_cell.y())
        {
            conversion_result = std::from_chars(conversion_result.ptr, end_ptr, cell_value);

            if (conversion_result.ec != std::errc())
                break;

            ++conversion_result.ptr; // skip the delimiting whitespace character

            if (cell_value > 0)
                grid_accessor.setValue(current_cell, cell_value);
        }

        if (conversion_result.ec != std::errc())
        {
            std::error_code ec = std::make_error_code(conversion_result.ec);
            logger.error() << "Parsing line " << line << " failed. Error: "
                           << ec.message();
            return OccupancyGrid::Ptr();
        }

        ++current_cell.x();
    }

    if (file_stream.bad())
    {
        logger.error() << "Error while reading " << filename
                       << " : " << strerror(errno);
        return OccupancyGrid::Ptr();
    }

    logger.info() << "Cells read: " << current_cell.asVec3i().str();

    return grid;
}

} // namespace gaden
