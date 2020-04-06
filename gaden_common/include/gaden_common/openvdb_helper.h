#ifndef GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED
#define GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED

#include <sstream>
#include <string>

#include <openvdb/math/Vec3.h>
#include <yaml-cpp/yaml.h>

namespace gaden {

template <typename T>
std::string toString(const openvdb::math::Vec3<T> &vector)
{
    return vector.str();
}

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

} // namespace gaden::openvdb_helper

#endif // GADEN_COMMON_OPENVDB_HELPER_H_INCLUDED
