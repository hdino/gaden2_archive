#include <gaden_preprocessing/png_exporter.h>

#include <vector>

#include <png++/png.hpp>
#include <rclcpp/logging.hpp>

#include <iostream>
#include <openvdb/Types.h>
namespace gaden {

void exportPng(OccupancyGrid::Ptr &grid, const std::string &png_file,
               rclcpp::Logger &logger, unsigned scale)
{
    // The PNG file is created in two steps:
    // 1) An unscaled 2D matrix is filled with the top view data
    // 2) The top view data is saved into the scaled image

    auto grid_accessor = grid->getAccessor();

    // Get the dimension and bounding box of the environment
    openvdb::Vec3i env_dimension_signed = grid->evalActiveVoxelDim().asVec3i();
    if (env_dimension_signed.x() < 0 || env_dimension_signed.y() < 0)
    {
        RCLCPP_ERROR(logger, "Environment has negative dimension. "
                             "Cannot export PNG.");
        return;
    }
    openvdb::math::Vec3<size_t> env_dimension = env_dimension_signed; // cast to unsigned type

    openvdb::CoordBBox env_bounding_box = grid->evalActiveVoxelBoundingBox();
    openvdb::Vec3i env_min = env_bounding_box.min().asVec3i();

    // We use a vector of vectors for interim storage of the top view data
    enum class Occupancy2D { Free, OccupiedGround, OccupiedAboveGround, Outlet };
    std::vector<std::vector<Occupancy2D>> data;
    data.resize(env_dimension.y(), std::vector<Occupancy2D>(env_dimension.x(), Occupancy2D::Free));
    // Note to access the vectors in correct order: data[y][x]

    // Iterate over all values in the grid that are "on",
    // i.e. have a value different from the background value
    for (auto it = grid->cbeginValueOn(); it.test(); ++it)
    {
        if (it.isVoxelValue()) // a voxel is a single element in the grid
        {
            openvdb::Coord coord = it.getCoord();
            size_t x = coord.x() - env_min.x();
            size_t y = coord.y() - env_min.y();
            size_t z = coord.z() - env_min.z();

            Occupancy2D &map_value = data.at(y).at(x);
            if (map_value == Occupancy2D::Outlet)
                continue;

            auto grid_value = grid_accessor.getValue(coord);
            Occupancy typed_value = static_cast<Occupancy>(grid_value);

            // An outlet overrides everything.
            if (typed_value == Occupancy::Outlet)
            {
                map_value = Occupancy2D::Outlet;
                continue;
            }

            if (typed_value == Occupancy::Occupied)
            {
                // Occupied ground cells override cells above ground.
                if (map_value == Occupancy2D::OccupiedGround)
                    continue;

                if (z == 0)
                    map_value = Occupancy2D::OccupiedGround;
                else
                    map_value = Occupancy2D::OccupiedAboveGround;
            }
            else
            {
                RCLCPP_ERROR_STREAM(logger, "Occupancy map has invalid value at"
                                    <<" x=" << x << " y=" << y << " z=" << z
                                    << " : " << grid_value);
                continue;
            }
        }
        else // a tile describes a larger area in the grid, but this is not used by GADEN
        {
            openvdb::CoordBBox bbox;
            it.getBoundingBox(bbox);
            RCLCPP_WARN(logger, "Grid tiles are not supported, yet");
        }
    }

    // Create, fill and save image
    openvdb::math::Vec3<size_t> image_dimension = scale * env_dimension;
    RCLCPP_INFO_STREAM(logger, "Scale: " << scale << " PNG dimension: "
                       << image_dimension.str());

    png::image<png::rgb_pixel> image(image_dimension.x(), image_dimension.y());

    png::rgb_pixel black(0, 0, 0);
    png::rgb_pixel white(255, 255, 255);
    png::rgb_pixel grey(128, 128, 128);
    png::rgb_pixel red(255, 0, 0);

    for (size_t x = 0; x < env_dimension.x(); ++x)
        for (size_t y = 0; y < env_dimension.y(); ++y)
        {
            Occupancy2D value = data.at(y).at(x);
            png::rgb_pixel pixel_colour;
            if (value == Occupancy2D::Free)
                pixel_colour = white;
            else if (value == Occupancy2D::OccupiedGround)
                pixel_colour = black;
            else if (value == Occupancy2D::OccupiedAboveGround)
                pixel_colour = grey;
            else
                pixel_colour = red;

            // repeat each value over scale * scale pixels
            for (uint32_t img_y = y * scale; img_y < (y + 1) * scale; ++img_y)
                for (uint32_t img_x = x * scale; img_x < (x + 1) * scale; ++img_x)
                    image[img_y][img_x] = pixel_colour;
        }

    image.write(png_file);
}

} // namespace gaden
