#include <iostream>

#include <cmath>
//#include <gaden_common/openvdb_box.hpp>

int main()
{
    std::cout << "Playground" << std::endl;

    double cell_size = 0.5;

//    gaden::open_vdb::BoundingBox box(Eigen::Vector3d(-10, -10, -10),
//                                     Eigen::Vector3d(10, 10, 10),
//                                     cell_size);

    //std::cout << std::hexfloat;
    for (unsigned i = 0; i < 23; ++i)
    {
        double x = -1.1 + i*0.1;
        //Eigen::Vector3d v(x, 0, 0);
        //openvdb::Coord cell = box.toCellCoord(v);
        //std::cout << x << " - " << cell.x() << std::endl;
        int cell = std::floor(x / cell_size);
        std::cout << x << " : " << cell << " : " << int(x / cell_size) << std::endl;
    }

    return 0;
}
