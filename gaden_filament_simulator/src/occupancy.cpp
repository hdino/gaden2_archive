#include <gaden_common/filesystem.h>
#include <gaden_common/inline_environment.hpp>
#include <gaden_common/occupancy_grid.h>
#include <gaden_common/openvdb_box.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/occupancy.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

#include <yaml-cpp/yaml.h>

namespace gaden {

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
            //Box box = getBox(object);
            open_vdb::Box box(object, cell_size);
            logger.info() << "Box:\n    " << box.toString(); // toString(box, 4);
            box.addToGrid(grid);
            //addBoxToGrid(box, grid);
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

    // save the size of the bounding box in world coordinates
    //Box bounding_box = getBox(yaml_environment["bounding_box"]);
//    open_vdb::BoundingBox bbox(bounding_box.origin,
//                               bounding_box.origin + bounding_box.size,
//                               grid->metaValue<double>("cell_size"));
    open_vdb::BoundingBox bounding_box(yaml_environment);
    bounding_box.addMetadataToGrid(grid);

    openvdb::io::File(config.occupancy_file).write({grid});

    return true;
}

} // namespace gaden
