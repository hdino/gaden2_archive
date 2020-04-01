#include <gaden_common/filesystem.h>

namespace gaden {

bool createDirectoriesIfNotExist(const std::string &path, rl::Logger &logger)
{
    // TODO catch fs exceptions
    std::filesystem::path directory(path);

    if (!std::filesystem::exists(directory))
    {
        if (!std::filesystem::create_directories(directory))
        {
            logger.error() << "Unable to create directory "
                           << directory.string();
            return false;
        }
    }
    else if (!std::filesystem::is_directory(directory))
    {
        logger.error() << "Given path " << directory.string()
                       << " is not a directory";
        return false;
    }

    return true;
}

} // namespace gaden
