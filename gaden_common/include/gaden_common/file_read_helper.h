#ifndef GADEN_COMMON_FILE_READ_HELPER_H_INCLUDED
#define GADEN_COMMON_FILE_READ_HELPER_H_INCLUDED

#include <fstream>

#include <rl_logging/logging_interface.hpp>

namespace gaden::file_read_helper {

bool getLineAndSkipStart(std::ifstream &file_stream, const char *start_string,
                         std::stringstream &output, rl::Logger &logger);

} // namespace gaden::file_read_helper

#endif // GADEN_COMMON_FILE_READ_HELPER_H_INCLUDED
