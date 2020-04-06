#ifndef GADEN_COMMON_TO_STRING_HPP_INCLUDED
#define GADEN_COMMON_TO_STRING_HPP_INCLUDED

#include <string>
#include <vector>

namespace gaden {

template <typename T>
std::string toString(const std::vector<T> &container, size_t indention = 0)
{
    std::string newline = "\n" + std::string(indention, ' ');

    std::string result;
    for (const T &item : container)
        result += newline + "- " +
                  toString(item, indention + 2);

    // remove leading newline from the first element
    if (!result.empty())
        result.erase(0, newline.length());

    return result;
}

/** boolean **/
inline std::string toString(bool value, size_t indention = 0)
{
    (void)indention;
    return (value ? "True" : "False");
}

/** uint types **/
inline std::string toString(uint8_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(uint16_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(uint32_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(uint64_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

/** int types **/
inline std::string toString(int8_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(int16_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(int32_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(int64_t value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

/** floating point types **/
inline std::string toString(float value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

inline std::string toString(double value, size_t indention = 0)
{
    (void)indention;
    return std::to_string(value);
}

} // namespace gaden

#endif // GADEN_COMMON_TO_STRING_HPP_INCLUDED
