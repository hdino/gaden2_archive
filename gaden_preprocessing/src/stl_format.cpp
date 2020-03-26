#include <gaden_common/filesystem.h>
#include <gaden_preprocessing/stl_format.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <rclcpp/logging.hpp>

namespace gaden {

//template <typename T>
//bool getFromStream(std::stringstream &stream, Eigen::Matrix<T, 3, 1> &vector)
//{
//    bool success = true;
//    success &= static_cast<bool>(stream >> vector[0]);
//    success &= static_cast<bool>(stream >> vector[1]);
//    success &= static_cast<bool>(stream >> vector[2]);
//    return success;
//}

template <typename T>
bool getFromStream(std::stringstream &stream, openvdb::math::Vec3<T> &vector)
{
    bool success = true;
    success &= static_cast<bool>(stream >> vector.x());
    success &= static_cast<bool>(stream >> vector.y());
    success &= static_cast<bool>(stream >> vector.z());
    return success;
}

bool getLineAndSkipStart(std::ifstream &file_stream, const char *start_string,
                         std::stringstream &output, rclcpp::Logger &logger)
{
    std::string line;
    if (!std::getline(file_stream, line))
    {
        if (file_stream.bad())
            RCLCPP_ERROR_STREAM(logger, "Error while reading STL file: " << strerror(errno));
        return false;
    }

    boost::algorithm::trim_left(line);

    if (line.rfind(start_string, 0) == 0)
    {
        output = std::stringstream(line);
        output.ignore(strlen(start_string));
        return true;
    }

    RCLCPP_ERROR_STREAM(logger, "Error while reading STL file: Invalid line: " << line);
    return false;
}

StlData readStlAscii(const std::string &filename, rclcpp::Logger logger)
{
    RCLCPP_INFO_STREAM(logger, "Attempting to read STL file: " << filename);

    std::vector<StlFacet> facets; // return value of this function

    // get the size of the STL file
    std::error_code error_code;
    auto file_size = std::filesystem::file_size(filename, error_code);
    if (error_code)
    {
        RCLCPP_ERROR_STREAM(logger, "Failed to get size of file " << filename
                            << " : " << error_code.message());
        return facets;
    }
    RCLCPP_INFO_STREAM(logger, "Size of the STL file: " << file_size << " Bytes");

    // open the STL file
    std::ifstream file_stream(filename.c_str());
    if (!file_stream.is_open())
    {
        RCLCPP_ERROR_STREAM(logger, "Failed to open " << filename
                            << " : " << strerror(errno));
        return facets;
    }

    // The evaluation of some ASCII STL files showed that the file size divided by 20
    // gives a good estimate of the number of lines. 7 lines form a facet.
    size_t estimated_number_of_facets = file_size / 20 / 7;
    RCLCPP_INFO_STREAM(logger, "Estimated number of facets in the STL file: "
                       << estimated_number_of_facets);
    facets.reserve(estimated_number_of_facets);

    // The actual data starts after the "solid" keyword in the STL.
    // Therefore, we skip all lines before "solid" occurs.
    bool found_solid = false;
    std::string line;
    while (std::getline(file_stream, line))
    {
        if (line.rfind("solid", 0) == 0)
        {
            found_solid = true;
            break;
        }
    }

    if (!found_solid)
    {
        if (file_stream.bad())
            RCLCPP_ERROR_STREAM(logger, "Error while reading STL file "
                                << filename << " : " << strerror(errno));
        else
            RCLCPP_ERROR_STREAM(logger, "Error while reading STL file "
                                << filename << " : Keyword 'solid' not found");
        facets.shrink_to_fit();
        return facets;
    }

    // In the STL file one or more facet blocks will follow
    bool file_valid = false;
    while (std::getline(file_stream, line))
    {
        boost::algorithm::trim_left(line);

        if (line.rfind("facet normal", 0) == 0)
        {
            StlFacet facet;
            std::stringstream stream(line);
            stream.ignore(strlen("facet normal") + 1);
            if (!getFromStream(stream, facet.facet_normal)) break;
            if (!getLineAndSkipStart(file_stream, "outer loop", stream, logger)) break;
            if (!getLineAndSkipStart(file_stream, "vertex ", stream, logger)) break;
            if (!getFromStream(stream, facet.vertices[0])) break;
            if (!getLineAndSkipStart(file_stream, "vertex ", stream, logger)) break;
            if (!getFromStream(stream, facet.vertices[1])) break;
            if (!getLineAndSkipStart(file_stream, "vertex ", stream, logger)) break;
            if (!getFromStream(stream, facet.vertices[2])) break;
            if (!getLineAndSkipStart(file_stream, "endloop", stream, logger)) break;
            if (!getLineAndSkipStart(file_stream, "endfacet", stream, logger)) break;

            //std::cout << stream.str() << std::endl;
            //std::cout << facet.vertex[2][0] << ", " << facet.vertex[2][1] << ", " << facet.vertex[2][2] << std::endl;
            //std::cout << facet.facet_normal[0] << ", " << facet.facet_normal[1] << ", " << facet.facet_normal[2] << std::endl;

            facets.push_back(std::move(facet));
        }
        else if (line.rfind("endsolid", 0) == 0)
        {
            file_valid = true;
            break;
        }
    }

    if (!file_valid)
    {
        if (file_stream.bad())
        {
            RCLCPP_ERROR_STREAM(logger, "Error while reading STL file "
                                << filename << " : " << strerror(errno));
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger, "Error while reading STL file "
                                << filename << ". Last valid line: " << line);
        }

        facets.clear();
        facets.shrink_to_fit();
        return facets;
    }

    facets.shrink_to_fit();
    RCLCPP_INFO_STREAM(logger, "Facets in the STL file: " << facets.size()
                       << ". Capacity after shrinking: " << facets.capacity());
    return facets;
}

} // namespace gaden
