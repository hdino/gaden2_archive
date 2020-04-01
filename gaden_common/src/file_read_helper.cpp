#include <gaden_common/file_read_helper.h>

#include <sstream>

#include <boost/algorithm/string.hpp>

namespace gaden::file_read_helper {

bool getLineAndSkipStart(std::ifstream &file_stream, const char *start_string,
                         std::stringstream &output, rl::Logger &logger)
{
    std::string line;
    if (!std::getline(file_stream, line))
    {
        if (file_stream.bad())
            logger.error() << "Error while reading from file: " << strerror(errno);
        return false;
    }

    boost::algorithm::trim_left(line);

    if (line.rfind(start_string, 0) == 0)
    {
        output = std::stringstream(line);
        output.ignore(strlen(start_string));
        return true;
    }

    logger.error() << "Error while reading from file: Invalid line: " << line;
    return false;
}

} // namespace gaden::file_read_helper
