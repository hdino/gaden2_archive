#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/filament.hpp>
#include <gaden_filament_simulator/filament_grid.hpp>

#include <cmath>

#include <iostream>
#include <gaden_common/eigen_helper.hpp>

namespace gaden {

int getNextPowerOf2(int value)
{
    int power = 1;
    while (power < value)
        power *= 2; // compiler should optimize this to power <<= 1
    return power;
}

void limitLowerBound(Eigen::Array3i &a, int limit)
{
    Eigen::Array3i mask = (a < limit).cast<int>();
    a = mask * limit + (1-mask) * a;
}

void limitUpperBound(Eigen::Array3i &a, const Eigen::Array3i &limit)
{
    Eigen::Array3i mask = (a > limit).cast<int>();
    a = mask * limit + (1-mask) * a;
}

FilamentGrid::FilamentGrid(
        double cell_size,
        std::shared_ptr<EnvironmentModel> environment_model)
    : cell_size_(cell_size)
    , cell_size_squared_(cell_size_ * cell_size_)
    , cell_size_half_squared_(0.25 * cell_size_squared_)
    , cell_size_vector_(Eigen::Vector3d::Constant(cell_size_))
    , cell_size_half_vector_(0.5 * cell_size_vector_)
    , env_model_(environment_model)
    , world_min_(env_model_->getEnvironmentMin())
    , world_max_(env_model_->getEnvironmentMax())
{
    Eigen::Vector3d world_size = world_max_ - world_min_;
    cell_count_ = (world_size.array() / cell_size_).ceil().cast<int>();
    octree_size_ = getNextPowerOf2(cell_count_.maxCoeff());
    std::cout << "Octree size: " << octree_size_ << std::endl;
    clear();
}

FilamentGrid::~FilamentGrid()
{}

void FilamentGrid::clear()
{
    // The octree doesn't have a clear method, so we need to fully destroy it.
    octree_.reset(); // free memory
    octree_ = std::make_unique<FilamentOctree>(octree_size_);
}

void FilamentGrid::add(Filament *filament)
{
    const Eigen::Vector3d &filament_position = filament->position;
    Eigen::Array3i filament_cell = getCellCoordinates(filament_position);

//    using namespace std;
//    cout << "##### Adding filament at " << toString(filament_position) << endl;
//    cout << "Filament cell: " << toString(filament_cell) << endl;

    double filament_range_pow2 = filament->getSquared3SigmaRadius();
    int filament_cell_range = std::ceil(filament_range_pow2 / cell_size_squared_);
    Eigen::Array3i cell_range = Eigen::Vector3i::Constant(filament_cell_range);

//    cout << "Filament R^2: " << filament_range_pow2 << " Cell range: " << toString(cell_range) << endl;

    Eigen::Vector3d filament_cell_centre = getCellCentrePosition(filament_cell);
    Eigen::Array3i cell_offset = -cell_range;
    for (; cell_offset[2] <= cell_range[2]; ++cell_offset[2])
    {
        for (; cell_offset[1] <= cell_range[1]; ++cell_offset[1])
        {
            for (; cell_offset[0] <= cell_range[0]; ++cell_offset[0])
            {
                Eigen::Vector3d current_cell_centre = filament_cell_centre + (cell_offset.cast<double>()*cell_size_).matrix();
                Eigen::Vector3d offset = current_cell_centre - filament_position;
                double distance_pow2 = offset.squaredNorm();

                if (filament_range_pow2 > distance_pow2 - cell_size_half_squared_
                        || (cell_offset == 0).all())
                {
                    if ((cell_offset < 0).any() || (cell_offset > cell_count_).any())
                        continue;

                    Eigen::Array3i current_cell = filament_cell + cell_offset;
                    std::vector<FilamentInfluence> &filaments =
                            octree_->operator ()(current_cell[0],
                                                 current_cell[1],
                                                 current_cell[2]);
                    filaments.emplace_back(filament, false); // TODO Check for obstacles
                }
            }
            cell_offset[0] = -cell_range[0];
        }
        cell_offset[1] = -cell_range[1];
    } // for

//    Eigen::Array3i start_cell = filament_cell - cell_range;
//    Eigen::Array3i end_cell = filament_cell + cell_range;

//    limitLowerBound(start_cell, 0);
//    limitUpperBound(end_cell, cell_count_);

//    Eigen::Array3i current_cell = start_cell;
//    for (; current_cell[2] <= end_cell[2]; ++current_cell[2])
//    {
//        for (; current_cell[1] <= end_cell[1]; ++current_cell[1])
//        {
//            for (; current_cell[0] <= end_cell[0]; ++current_cell[0])
//            {
//                Eigen::Vector3d current_cell_centre =
//                        getCellCentrePosition(current_cell);

//                Eigen::Vector3d offset = current_cell_centre - filament_position;

//                double distance_pow2 = offset.squaredNorm();

////                cout << "Checking cell: " << toString(current_cell) << endl;
////                cout << "Cell centre: " << toString(current_cell_centre) << endl;
////                cout << "Pos offset: " << toString(offset) << "   d^2: " << distance_pow2 << endl;

//                if (filament_range_pow2 > distance_pow2 - cell_size_half_squared_
//                        || (current_cell == filament_cell).all())
//                {
////                    cout << "ADD FILAMENT" << endl;
//                    std::vector<FilamentInfluence> &filaments =
//                            octree_->operator ()(current_cell[0],
//                                                 current_cell[1],
//                                                 current_cell[2]);
//                    filaments.emplace_back(filament, false); // TODO Check for obstacles
//                }
//            } // for x
//            current_cell[0] = start_cell[0];
//        } // for y
//        current_cell[1] = start_cell[1];
//    } // for z
}

//Eigen::Vector3d
//FilamentGrid::getCellPosition(
//        const Eigen::Array3i &cell_coordinates) const
//{
//    return (cell_coordinates.cast<double>() * cell_size_).matrix()
//            + world_min_;
//}

//Eigen::Vector3d
//FilamentGrid::getCellCentrePosition(
//        const Eigen::Array3i &cell_coordinates) const
//{
//    return getCellPosition(cell_coordinates) + cell_size_half_vector_;
//}

//Eigen::Vector3d
//FilamentGrid::getCellUpperPosition(
//        const Eigen::Array3i &cell_coordinates) const
//{
//    return getCellPosition(cell_coordinates) + cell_size_vector_;
//}

//bool FilamentGrid::isInCell(const Eigen::Vector3d &world_position,
//                            const Eigen::Array3i &cell_coordinates) const
//{
//    return (getCellCoordinates(world_position) == cell_coordinates).all();
//}

} // namespace gaden
