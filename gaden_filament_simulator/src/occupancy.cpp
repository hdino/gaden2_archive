#include <gaden_common/filesystem.h>
#include <gaden_common/inline_environment.hpp>
#include <gaden_common/occupancy_grid.h>
#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/occupancy.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

#include <yaml-cpp/yaml.h>

namespace gaden {

void addBoxToGrid(const Box &box, OccupancyGrid::Ptr &grid)
{
    double cell_size = grid->metaValue<double>("cell_size");

    openvdb::Vec3i box_origin = box.origin / cell_size;
    openvdb::Vec3i box_size = box.size / cell_size;
    openvdb::Vec3i box_end = box_origin + box_size;

    //std::cout << "Origin: " << toString(box_origin) << std::endl;
    //std::cout << "Size: " << toString(box_size) << std::endl;
    //std::cout << "End: " << toString(box_end) << std::endl;

    auto grid_accessor = grid->getAccessor();

    openvdb::Coord xyz;
    for (xyz.z() = box_origin.z(); xyz.z() < box_end.z(); ++xyz.z())
        for (xyz.y() = box_origin.y(); xyz.y() < box_end.y(); ++xyz.y())
            for (xyz.x() = box_origin.x(); xyz.x() < box_end.x(); ++xyz.x())
                grid_accessor.setValue(xyz, 1);
}

OccupancyGrid::Ptr generateInlineEnvironment(const YAML::Node &env, rl::Logger &logger)
{
    logger.info("Generating environment from inline data");

    double cell_size = env["cell_size"].as<double>();
    logger.info() << "Cell size: " << cell_size;

    OccupancyGrid::Ptr grid = createGrid(cell_size);

    YAML::Node inline_env = env["inline_environment"];

    for (const YAML::Node &object : inline_env["objects"])
    {
        std::string object_type = object["type"].as<std::string>();
        if (object_type == "box")
        {
            Box box = getBox(object);
            logger.info() << "Box:\n    " << toString(box, 4);
            addBoxToGrid(box, grid);
        }
        else
            logger.warn() << "Unrecognised object type: " << object_type;
    }

    return grid;
}

bool generateOccupancyFile(const SimulatorConfig &config, rl::Logger &logger)
{
    std::filesystem::create_directories(config.occupancy_file.parent_path());

    YAML::Node yaml_config = YAML::LoadFile(config.base_config_file);

    YAML::Node yaml_environment = yaml_config["environment"];

    std::string environment_format = yaml_environment["format"].as<std::string>();

    OccupancyGrid::Ptr grid;
    if (environment_format == "inline")
        grid = generateInlineEnvironment(yaml_environment, logger);
    else
    {
        logger.error() << "Invalid environment format: " << environment_format;
        return false;
    }

    double cell_size = grid->metaValue<double>("cell_size");
    Box bounding_box = getBox(yaml_environment["bounding_box"]);
    openvdb::Vec3i bounding_box_origin = bounding_box.origin / cell_size;
    openvdb::Vec3i bounding_box_size = bounding_box.size / cell_size;

    grid->insertMeta("bounding_box_origin", openvdb::Vec3IMetadata(bounding_box_origin));
    grid->insertMeta("bounding_box_size", openvdb::Vec3IMetadata(bounding_box_size));

    openvdb::io::File(config.occupancy_file).write({grid});

    return true;
}

} // namespace gaden
