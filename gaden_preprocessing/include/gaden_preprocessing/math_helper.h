#ifndef GADEN_PREPROCESSING_MATH_HELPER_H_INCLUDED
#define GADEN_PREPROCESSING_MATH_HELPER_H_INCLUDED

#include <algorithm>
#include <type_traits>

namespace gaden {

template <typename E>
constexpr auto toUnderlying(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}

template <typename TValue,
          typename TTolerance = std::conditional_t<std::is_same_v<TValue, float>, float, double>>
class Tolerant
{
public:
    // C++20 would allow: Tolerant(T value, std::type_identity_t<T> tolerance = 1e-4)
    Tolerant(TValue value, TTolerance tolerance = 1e-4)
        : value_(value)
        , tolerance_(tolerance)
    {
        static_assert(std::is_same_v<TValue, float> || std::is_same_v<TValue, double>,
                      "Only float and double supported");
    }

    bool operator ==(TValue other) const
    {
        return std::abs(value_ - other) < tolerance_;
    }

private:
    TValue value_;
    TTolerance tolerance_;
};

//template <typename T>
//std::enable_if_t<std::is_floating_point_v<T>, T> isEqualTolerance(T a, T b, T tolerance = 1e-4)
//{
//    return std::abs(a - b) < tolerance;
//}

//template <typename T>
//struct MinMax
//{
//    MinMax(T min_in, T max_in) : min(min_in), max(max_in) {}
//    T min;
//    T max;
//};

//template <typename T>
//MinMax<T> getMinMax(std::initializer_list<T> &&il)
//{
//    return MinMax<T>(std::min(std::forward<std::initializer_list<T>>(il)),
//                     std::max(std::forward<std::initializer_list<T>>(il)));
//}

} // namespace gaden

#endif // GADEN_PREPROCESSING_MATH_HELPER_H_INCLUDED
