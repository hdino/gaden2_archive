#ifndef GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED
#define GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED

#include <string>

#include <Eigen/Core>

namespace YAML {
class Node;
}

namespace gaden {

template <typename T>
inline std::string toString(const Eigen::Matrix<T, 3, 1> &vec, size_t indention = 0)
{
    (void)indention;
    return "[" + std::to_string(vec[0]) + ", "
               + std::to_string(vec[1]) + ", "
               + std::to_string(vec[2]) + "]";
}

} // namespace gaden

namespace gaden::eigen_helper {

Eigen::Vector3d getVector3dFromYaml(const YAML::Node &sequence);

} // namespace gaden::eigen_helper

#endif // GADEN_COMMON_EIGEN_HELPER_HPP_INCLUDED
