#include <gaden_common/filesystem.h>

#include <rclcpp/logging.hpp>

namespace gaden {

bool createDirectoriesIfNotExist(const std::string &path, rclcpp::Logger &logger)
{
    // TODO catch fs exceptions
    std::filesystem::path directory(path);

    if (!std::filesystem::exists(directory))
    {
        if (!std::filesystem::create_directories(directory))
        {
            RCLCPP_ERROR_STREAM(logger, "Unable to create directory "
                                << directory.string());
            return false;
        }
    }
    else if (!std::filesystem::is_directory(directory))
    {
        RCLCPP_ERROR_STREAM(logger, "Given path " << directory.string()
                            << " is not a directory");
        return false;
    }

    return true;
}

} // namespace gaden
