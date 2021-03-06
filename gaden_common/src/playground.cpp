#include <iostream>

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>

//#include <gaden_common/cache_grid.hpp>
#include <gaden_common/eigen_helper.hpp>
#include <rl_logging/std_logging.hpp>

#include <octree/octree.h>
#include <gaden_common/object_pool.hpp>

double generateValue(const Eigen::Vector3d &p)
{
    std::cout << "Generating at " << gaden::toString(p) << std::endl;
    return p[0] + p[1] + p[2];
}

class InstanceCounter
{
public:
    InstanceCounter()
    {
        ++counter;
    }

    virtual ~InstanceCounter()
    {
        --counter;
    }

    inline static unsigned getCounter() { return counter.load(); }

private:
    static inline std::atomic<unsigned> counter = 0;
};

int main()
{
    std::cout << "Playground" << std::endl;

    Octree<int> octree(128);
    gaden::ObjectPool<int> pool;

    //rl::Logger logger = rl::logging::StdLogger::create("main");

//    double cell_size = 0.5;

//    gaden::CacheGrid<openvdb::DoubleGrid> grid(cell_size, &generateValue);

//    Eigen::Vector3d pos(1, 1, 1);
//    std::cout << grid.getValue(pos) << std::endl;
//    std::cout << grid.getValue(pos) << std::endl;
//    pos[0] = 1.2;
//    std::cout << grid.getValue(pos) << std::endl;
//    pos[0] = 1.5;
//    std::cout << grid.getValue(pos) << std::endl;
//    std::cout << grid.getValue(pos) << std::endl;
//    grid.clear();
//    std::cout << grid.getValue(pos) << std::endl;

    return 0;
}
