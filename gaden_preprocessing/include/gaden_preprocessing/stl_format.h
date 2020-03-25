#ifndef GADEN_PREPROCESSING_STL_FORMAT_H_INCLUDED
#define GADEN_PREPROCESSING_STL_FORMAT_H_INCLUDED

#include <string>

#include <rclcpp/logger.hpp>

#include "stl_data.h"

namespace gaden {

StlData readStlAscii(const std::string &filename, rclcpp::Logger logger);

} // namespace gaden

#endif // GADEN_PREPROCESSING_STL_FORMAT_H_INCLUDED
