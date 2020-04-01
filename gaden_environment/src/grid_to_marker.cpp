#include <gaden_common/ros_type_helper.h>
#include <gaden_environment/grid_to_marker.h>

namespace gaden {

visualization_msgs::msg::MarkerArray
getAsMarkerArray(OccupancyGrid::Ptr grid,
                 const builtin_interfaces::msg::Time &stamp,
                 const std::string &frame_id,
                 rl::Logger &log)
{
    auto grid_accessor = grid->getAccessor();
    double cell_size = grid->metaValue<double>("cell_size");
    log.info() << "Cell size of imported grid: " << cell_size;

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "environment_visualization";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = ros_type::DefaultOrientation::get();
    marker.scale = ros_type::getVector3(cell_size);

    std_msgs::msg::ColorRGBA colour_occupied = ros_type::getColor(0.8, 0.8, 0.8);
    std_msgs::msg::ColorRGBA colour_outlet = ros_type::getColor(0.9, 0.1, 0.1);

    // Iterate over all values in the grid that are "on",
    // i.e. have a value different from the background value
    for (auto it = grid->cbeginValueOn(); it.test(); ++it)
    {
        if (it.isVoxelValue()) // a voxel is a single element in the grid
        {
            openvdb::Coord coord = it.getCoord();
            auto grid_value = grid_accessor.getValue(coord);
            Occupancy typed_value = static_cast<Occupancy>(grid_value);

            openvdb::Vec3d xyz = coord.asVec3d();
            xyz += 0.5;
            xyz *= cell_size;

            marker.pose.position = ros_type::getPoint(xyz.x(), xyz.y(), xyz.z());

            if (typed_value == Occupancy::Occupied)
                marker.color = colour_occupied;
            else if (typed_value == Occupancy::Outlet)
                marker.color = colour_outlet;
            else
                continue;

            marker_array.markers.push_back(marker);
            ++marker.id;
        }
        else // a tile describes a larger area in the grid, but this is not used by GADEN
        {
            openvdb::CoordBBox bbox;
            it.getBoundingBox(bbox);
            log.warn("Grid contains a tile, but this is not supported (yet)");
        }
    }

    return marker_array;
}

} // namespace gaden


//                        //Center of the cell
//                        new_marker.pose.position.x = env_min_x + ( (x_idx + 0.5) * cell_size);
//                        new_marker.pose.position.y = env_min_y + ( (y_idx + 0.5) * cell_size);
//                        new_marker.pose.position.z = env_min_z + ( (z_idx + 0.5) * cell_size);


/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays it.
 * As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet.
 * This function creates a cube marker for every occupied cell, with the corresponding dimensions
*/
//void loadEnvironment(visualization_msgs::MarkerArray &env_marker)
//{



//    //open file
//    std::ifstream infile(occupancy3D_data.c_str());
//    std::string line;

//    //Read the 4 Header lines
//    {
//        //Line 1 (min values of environment)
//        std::getline(infile, line);
//        size_t pos = line.find(" ");
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_min_x = atof(line.substr(0, pos).c_str());
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_min_y = atof(line.substr(0, pos).c_str());
//        env_min_z = atof(line.substr(pos+1).c_str());

//        //Line 2 (max values of environment)
//        std::getline(infile, line);
//        pos = line.find(" ");
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_max_x = atof(line.substr(0, pos).c_str());
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_max_y = atof(line.substr(0, pos).c_str());
//        env_max_z = atof(line.substr(pos+1).c_str());

//        //Line 3 (Num cells on eahc dimension)
//        std::getline(infile, line);
//        pos = line.find(" ");
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_cells_x = atoi(line.substr(0, pos).c_str());
//        line.erase(0, pos+1);
//        pos = line.find(" ");
//        env_cells_y = atof(line.substr(0, pos).c_str());
//        env_cells_z = atof(line.substr(pos+1).c_str());

//        //Line 4 cell_size (m)
//        std::getline(infile, line);
//        pos = line.find(" ");
//        cell_size = atof(line.substr(pos+1).c_str());

//        if (verbose) ROS_INFO("[env]Env dimensions (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)",env_min_x, env_min_y, env_min_z, env_max_x, env_max_y, env_max_z );
//        if (verbose) ROS_INFO("[env]Env size in cells     (%d,%d,%d) - with cell size %f [m]",env_cells_x,env_cells_y,env_cells_z, cell_size);
//    }


//    //Read file line by line
//    int x_idx = 0;
//    int y_idx = 0;
//    int z_idx = 0;
//    int cell_idx = 0;

//    while (std::getline(infile, line))
//    {
//        std::stringstream ss(line);

//        if (line == ";")
//        {
//            //New Z-layer
//            z_idx++;
//            x_idx = 0;
//            y_idx = 0;
//        }
//        else
//        {   //New line with constant x_idx and all the y_idx values
//            while (!ss.fail())
//            {
//                double f;
//                ss >> f;        //get one double value
//                if (!ss.fail())
//                {
//                    // Occupie cell or Outlet
//                    if (f == 1.0 || f == 2.0)
//                    {
//                        //Add a new cube marker for this occupied cell
//                        visualization_msgs::Marker new_marker;
//                        new_marker.header.frame_id = fixed_frame;
//                        new_marker.header.stamp = ros::Time::now();
//                        new_marker.ns = "environment_visualization";
//                        new_marker.id = cell_idx;                          //unique identifier
//                        new_marker.type = visualization_msgs::Marker::CUBE;
//                        new_marker.action = visualization_msgs::Marker::ADD;

//                        //Center of the cell
//                        new_marker.pose.position.x = env_min_x + ( (x_idx + 0.5) * cell_size);
//                        new_marker.pose.position.y = env_min_y + ( (y_idx + 0.5) * cell_size);
//                        new_marker.pose.position.z = env_min_z + ( (z_idx + 0.5) * cell_size);
//                        new_marker.pose.orientation.x = 0.0;
//                        new_marker.pose.orientation.y = 0.0;
//                        new_marker.pose.orientation.z = 0.0;
//                        new_marker.pose.orientation.w = 1.0;

//                        //Size of the cell
//                        new_marker.scale.x = cell_size;
//                        new_marker.scale.y = cell_size;
//                        new_marker.scale.z = cell_size;

//                        //Color
//                        if (f == 1.0)
//                        {
//                            new_marker.color.r = 0.8f;
//                            new_marker.color.g = 0.8f;
//                            new_marker.color.b = 0.8f;
//                            new_marker.color.a = 1.0;
//                        }
//                        else
//                        {
//                            new_marker.color.r = 0.9f;
//                            new_marker.color.g = 0.1f;
//                            new_marker.color.b = 0.1f;
//                            new_marker.color.a = 1.0;
//                        }

//                        env_marker.markers.push_back(new_marker);
//                    }
//                    y_idx++;
//                    cell_idx++;
//                }
//            }

//            //Line has ended
//            x_idx++;
//            y_idx = 0;
//        }
//    }

//    //End of file.
//}
