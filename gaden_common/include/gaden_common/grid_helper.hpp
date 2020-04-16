#ifndef GADEN_COMMON_GRID_HELPER_HPP_INCLUDED
#define GADEN_COMMON_GRID_HELPER_HPP_INCLUDED

#include <Eigen/Core>
#include <openvdb/Types.h>

namespace gaden::grid_helper {

inline openvdb::Coord getCellCoordinates(Eigen::Vector3d world_coordinates, double cell_size)
{
    world_coordinates = (world_coordinates / cell_size).array().floor();
    return openvdb::Coord(world_coordinates[0],
                          world_coordinates[1],
                          world_coordinates[2]);
}

template <typename T>
openvdb::Vec3i getCellCoordinates(openvdb::math::Vec3<T> vector, double cell_size)
{
    vector /= cell_size;
    vector.x() = std::floor(vector.x());
    vector.y() = std::floor(vector.y());
    vector.z() = std::floor(vector.z());
    return vector;
}

template <typename T, size_t N>
std::array<openvdb::Vec3i, N>
getCellCoordinates(const std::array<openvdb::math::Vec3<T>, N> &continuous_coordinates,
                   double cell_size)
{
    std::array<openvdb::Vec3i, N> cell_coordinates;
    for (size_t i = 0; i < N; ++i)
        cell_coordinates[i] = getCellCoordinates(continuous_coordinates[i], cell_size);
    return cell_coordinates;
}

//template <typename T>
//openvdb::Vec3i getCellCoordinates(const Eigen::Matrix<T, 3, 1> &vector, double cell_size)
//{
//    Eigen::Matrix<T, 3, 1> temp = (vector / cell_size).array().floor();
//    return openvdb::Vec3i(temp[0], temp[1], temp[2]);
//}

template <typename T>
openvdb::math::Vec3<T> getGridDeviation(const openvdb::math::Vec3<T> &continuous_coordinates,
                                        double cell_size)
{
    return openvdb::math::Vec3<T>(std::fmod(continuous_coordinates.x(), cell_size),
                                  std::fmod(continuous_coordinates.y(), cell_size),
                                  std::fmod(continuous_coordinates.z(), cell_size));
}

} // namespace gaden::grid_helper

#endif // GADEN_COMMON_GRID_HELPER_HPP_INCLUDED
