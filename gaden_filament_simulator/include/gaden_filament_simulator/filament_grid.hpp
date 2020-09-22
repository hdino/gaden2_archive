#ifndef GADEN_SIMULATOR_FILAMENT_GRID_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_GRID_HPP_INCLUDED

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <octree/octree.h>

//#include <gaden_common/object_pool.hpp>

template <typename T, int AS>
class Octree;

namespace gaden {

class EnvironmentModel;
class Filament;

struct FilamentInfluence
{
    FilamentInfluence() {}

    FilamentInfluence(Filament *filament,
                      bool need_to_check_for_obstacle)
        : filament(filament)
        , need_to_check_for_obstacle(need_to_check_for_obstacle)
    {}

    Filament *filament;
    bool need_to_check_for_obstacle;
};

class FilamentGrid
{
public:
    FilamentGrid(double cell_size,
                 std::shared_ptr<EnvironmentModel> environment_model);

    ~FilamentGrid();

    void add(Filament *filament);
    void clear();

    inline const std::vector<FilamentInfluence> &
    getFilamentsAt(const Eigen::Vector3d &world_position)
    {
        Eigen::Array3i cell_coordinates = getCellCoordinates(world_position);
        return octree_->at(cell_coordinates[0],
                           cell_coordinates[1],
                           cell_coordinates[2]);
    }

private:
    using FilamentOctree = Octree<std::vector<FilamentInfluence>, 1>;

    inline Eigen::Array3i
    getCellCoordinates(const Eigen::Vector3d &world_position) const
    {
        // The Octree uses positive coordinates, therefore a simple cast to int
        // should do the right thing, no floor needed.
        return ((world_position - world_min_).array() / cell_size_).cast<int>();
    }

    inline Eigen::Vector3d
    getCellPosition(const Eigen::Array3i &cell_coordinates) const
    {
        return (cell_coordinates.cast<double>() * cell_size_).matrix()
                + world_min_;
    }

    inline Eigen::Vector3d
    getCellCentrePosition(const Eigen::Array3i &cell_coordinates) const
    {
        return getCellPosition(cell_coordinates) + cell_size_half_vector_;
    }

//    Eigen::Vector3d
//    getCellUpperPosition(const Eigen::Array3i &cell_coordinates) const;

//    bool isInCell(const Eigen::Vector3d &world_position,
//                  const Eigen::Array3i &cell_coordinates) const;

    double cell_size_;
    double cell_size_squared_;
    double cell_size_half_squared_;
    Eigen::Vector3d cell_size_vector_;
    Eigen::Vector3d cell_size_half_vector_;

    std::shared_ptr<EnvironmentModel> env_model_;
    Eigen::Vector3d world_min_;
    Eigen::Vector3d world_max_;
    Eigen::Array3i cell_count_;

    //ObjectPool<std::vector<Filament *>> vector_pool_;
    int octree_size_;
    std::unique_ptr<FilamentOctree> octree_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_GRID_HPP_INCLUDED
