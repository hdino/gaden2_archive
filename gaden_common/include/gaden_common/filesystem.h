#ifndef GADEN_COMMON_FILESYSTEM_H_INCLUDED
#define GADEN_COMMON_FILESYSTEM_H_INCLUDED

// include std::filesystem
#if __has_include(<filesystem>)
    #include <filesystem>
#else
    #include <experimental/filesystem>
    namespace std {
        namespace filesystem = experimental::filesystem;
    } // namespace std
#endif

#include <string>

#include <rl_logging/logging_interface.hpp>

namespace gaden {

bool createDirectoriesIfNotExist(const std::string &path, rl::Logger &logger);

} // namespace gaden

#endif // GADEN_COMMON_FILESYSTEM_H_INCLUDED
