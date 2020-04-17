#ifndef GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED
#define GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED

#include <sstream>
#include <string>

#include <Eigen/Core>
#include <openvdb/Types.h>
#include <yaml-cpp/yaml.h>

namespace gaden {

// The return value of initialiseOpenVdb has no meaning, it's always true and
// just there to use the function in a constructor's member initialiser list.
bool initialiseOpenVdb();

template <typename T>
std::string toString(const openvdb::math::Vec3<T> &vector)
{
    return vector.str();
}

std::string toString(const openvdb::Coord &coord, size_t indention = 0);

} // namespace gaden

namespace gaden::openvdb_helper {

template <typename T>
bool getFromStream(std::stringstream &stream, openvdb::math::Vec3<T> &vector)
{
    bool success = true;
    success &= static_cast<bool>(stream >> vector.x());
    success &= static_cast<bool>(stream >> vector.y());
    success &= static_cast<bool>(stream >> vector.z());
    return success;
}

template <typename T>
openvdb::math::Vec3<T> getVec3FromYaml(const YAML::Node &sequence)
{
    openvdb::math::Vec3<T> vector;
    vector.x() = sequence[0].as<T>();
    vector.y() = sequence[1].as<T>();
    vector.z() = sequence[2].as<T>();
    return vector;
}

template <typename T>
openvdb::math::Vec3<T> fromEigen(const Eigen::Matrix<T, 3, 1> &v)
{
    return openvdb::math::Vec3<T>(v[0], v[1], v[2]);
}

template <typename T>
Eigen::Matrix<T, 3, 1> toEigen(const openvdb::math::Vec3<T> &v)
{
    return Eigen::Matrix<T, 3, 1>(v.x(), v.y(), v.z());
}

//template <typename TOpenVdb, typename TEigen>
//void toEigen(const openvdb::math::Vec3<TOpenVdb> &in,
//             Eigen::Matrix<TEigen, 3, 1> &out)
//{
//    out[0] = in[0];
//    out[1] = in[1];
//    out[2] = in[2];
//}

} // namespace gaden::openvdb_helper

#endif // GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED
