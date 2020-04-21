#ifndef GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED
#define GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED

#include <sstream>
#include <string>

#include <Eigen/Core>

namespace YAML {
class Node;
}

namespace gaden {

namespace internal {

template <typename Derived>
std::string toStringFromEigenVec3Like(const Eigen::DenseBase<Derived> &vec, size_t indention)
{
    static_assert((Derived::RowsAtCompileTime == 1 && Derived::ColsAtCompileTime == 3) ||
                  (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1));

    (void)indention;
    return "[" + std::to_string(vec[0]) + ", "
               + std::to_string(vec[1]) + ", "
               + std::to_string(vec[2]) + "]";
}

} // namespace internal

template <typename T>
inline std::string toString(const Eigen::Matrix<T, 3, 1> &vec, size_t indention = 0)
{
    return internal::toStringFromEigenVec3Like(vec, indention);
}

template <typename T>
inline std::string toString(const Eigen::Array<T, 3, 1> &vec, size_t indention = 0)
{
    return internal::toStringFromEigenVec3Like(vec, indention);
}

} // namespace gaden

namespace gaden::eigen_helper {

Eigen::Vector3d getVector3dFromYaml(const YAML::Node &sequence);

template <typename T>
bool getFromStream(std::stringstream &stream, Eigen::Matrix<T, 3, 1> &vector)
{
    bool success = true;
    success &= static_cast<bool>(stream >> vector[0]);
    success &= static_cast<bool>(stream >> vector[1]);
    success &= static_cast<bool>(stream >> vector[2]);
    return success;
}

} // namespace gaden::eigen_helper

#endif // GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED
