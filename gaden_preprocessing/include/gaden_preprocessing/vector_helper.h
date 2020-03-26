#ifndef GADEN_PREPROCESSING_VECTOR_HELPER_H_INCLUDED
#define GADEN_PREPROCESSING_VECTOR_HELPER_H_INCLUDED

#include <algorithm>
#include <type_traits>

#include <Eigen/Core>
#include <openvdb/Types.h>

namespace gaden {

namespace internal {
auto compareX = [](const auto &a, const auto &b) { return a.x() < b.x(); };
auto compareY = [](const auto &a, const auto &b) { return a.y() < b.y(); };
auto compareZ = [](const auto &a, const auto &b) { return a.z() < b.z(); };
} // namespace internal

template <typename T>
struct MinMaxVector
{
    openvdb::math::Vec3<T> min;
    openvdb::math::Vec3<T> max;
};

template <typename TContainer, typename BaseType = typename TContainer::value_type::value_type>
MinMaxVector<typename TContainer::value_type::value_type> getElementWiseMinMax(const TContainer &input)
{
    //static_assert(std::is_same_v<TContainer::value_type, >);

    auto min_max_x = std::minmax_element(input.begin(), input.end(), internal::compareX);
    auto min_max_y = std::minmax_element(input.begin(), input.end(), internal::compareY);
    auto min_max_z = std::minmax_element(input.begin(), input.end(), internal::compareZ);

    MinMaxVector<typename TContainer::value_type::value_type> result;
    result.min.x() = min_max_x.first->x();
    result.min.y() = min_max_y.first->y();
    result.min.z() = min_max_z.first->z();

    result.max.x() = min_max_x.second->x();
    result.max.y() = min_max_y.second->y();
    result.max.z() = min_max_z.second->z();

    return result;
}

template <typename TContainer>
openvdb::math::Vec3<typename TContainer::value_type::value_type> getElementWiseMax(const TContainer &input)
{
    auto max_x = std::max_element(input.begin(), input.end(), internal::compareX);
    auto max_y = std::max_element(input.begin(), input.end(), internal::compareY);
    auto max_z = std::max_element(input.begin(), input.end(), internal::compareZ);

    return openvdb::math::Vec3<typename TContainer::value_type::value_type>(
        max_x->x(), max_y->y(), max_z->z());
}

template <typename T>
Eigen::Vector3d toEigenVector3d(const openvdb::math::Vec3<T> &vector)
{
    return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

} // namespace gaden

#endif // GADEN_PREPROCESSING_VECTOR_HELPER_H_INCLUDED
