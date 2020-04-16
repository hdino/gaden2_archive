#include <gaden_common/grid_helper.hpp>
#include <gaden_common/math_helper.h>
#include <gaden_preprocessing/stl_to_grid.h>
#include <gaden_preprocessing/vector_helper.h>

#include <queue>

#include <Eigen/Geometry>

namespace gaden {

void addStlToGrid(const std::vector<StlFacet> &facets,
                  OccupancyGrid::Ptr &grid,
                  Occupancy occupancy_type,
                  double cell_size)
{
    double cell_size_half = 0.5 * cell_size;

    auto grid_accessor = grid->getAccessor();

    for (const StlFacet &facet : facets)
    {
        auto vertex_cell_coordinates = grid_helper::getCellCoordinates(facet.vertices, cell_size);

        auto cell = getElementWiseMinMax(vertex_cell_coordinates);

        openvdb::Vec3s continuous_max = getElementWiseMax(facet.vertices);
        openvdb::Vec3s continuous_max_grid_deviation = grid_helper::getGridDeviation(continuous_max, cell_size);

        Vec3Bool limit(
            Tolerant(continuous_max_grid_deviation.x()) == 0
                || Tolerant(continuous_max_grid_deviation.x()) == cell_size,
            Tolerant(continuous_max_grid_deviation.y()) == 0
                || Tolerant(continuous_max_grid_deviation.y()) == cell_size,
            Tolerant(continuous_max_grid_deviation.z()) == 0
                || Tolerant(continuous_max_grid_deviation.z()) == cell_size);

        processVertex(grid_accessor, vertex_cell_coordinates[0], cell.max, limit, occupancy_type);
        processVertex(grid_accessor, vertex_cell_coordinates[1], cell.max, limit, occupancy_type);
        processVertex(grid_accessor, vertex_cell_coordinates[2], cell.max, limit, occupancy_type);

        openvdb::Vec3i xyz;
        for (xyz.x() = cell.min.x(); xyz.x() <= cell.max.x(); ++xyz.x())
            for (xyz.y() = cell.min.y(); xyz.y() <= cell.max.y(); ++xyz.y())
                for (xyz.z() = cell.min.z(); xyz.z() <= cell.max.z(); ++xyz.z())
                {
                    // check if the triangle goes through this cell
                    if (isPointInTriangle(Eigen::Vector3d(xyz.x() * cell_size + cell_size_half,
                                                          xyz.y() * cell_size + cell_size_half,
                                                          xyz.z() * cell_size + cell_size_half),
                                          toEigenVector3d(facet.vertices[0]),
                                          toEigenVector3d(facet.vertices[1]),
                                          toEigenVector3d(facet.vertices[2]),
                                          isParallelToBasis(facet.facet_normal),
                                          cell_size))
                    {
                        processVertex(grid_accessor, xyz, cell.max, limit, occupancy_type);
                    }
                }
    } // for (facet : facets)
}

//        std::cout << "Vertices:"
//                  << "\n  " << facet.vertices[0].str()
//                  << "\n  " << facet.vertices[1].str()
//                  << "\n  " << facet.vertices[2].str()
//                  << "\n  max: " << continuous_max.str() << std::endl;

void processVertex(OccupancyGrid::Accessor &grid_accessor,
                   const openvdb::Vec3i &vertex_cell_coordinates,
                   const openvdb::Vec3i &max_vertex_cell_coordinates,
                   const Vec3Bool &limit,
                   Occupancy occupancy_type)
{
    openvdb::Coord grid_coord(vertex_cell_coordinates);
    auto occupancy_value = toUnderlying(occupancy_type);

    if ((    (limit.x() && vertex_cell_coordinates.x() == max_vertex_cell_coordinates.x())
          || (limit.y() && vertex_cell_coordinates.y() == max_vertex_cell_coordinates.y())
          || (limit.z() && vertex_cell_coordinates.z() == max_vertex_cell_coordinates.z()))
        && (grid_accessor.getValue(grid_coord) != toUnderlying(Occupancy::Occupied)))
    {
        // TODO
        //if (occupancy_type == Occupancy::Occupied)
        //    occupancy_value = 4; // TODO Value 4 should have a name / assert that it's gone after preprocessing
    }
    grid_accessor.setValue(grid_coord, occupancy_value);
}

bool isParallelToBasis(const openvdb::Vec3s &vector)
{
    Tolerant x(vector.x());
    Tolerant y(vector.y());
    Tolerant z(vector.z());
    return (x == 0 && y == 0) ||
           (x == 0 && z == 0) ||
           (y == 0 && z == 0);
}

bool isPointInTriangle(const Eigen::Vector3d& query_point,
                       const Eigen::Vector3d& triangle_vertex_0,
                       const Eigen::Vector3d& triangle_vertex_1,
                       const Eigen::Vector3d& triangle_vertex_2,
                       bool parallel, double cell_size)
{
    // u=P2−P1
    Eigen::Vector3d u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    Eigen::Vector3d v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    Eigen::Vector3d n = u.cross(v);
    bool anyProjectionInTriangle = false;
    std::vector<Eigen::Vector3d> cube = cubePoints(query_point, cell_size);
    for (const Eigen::Vector3d &vec : cube)
    {
        // w=P−P1
        Eigen::Vector3d w = vec - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        double gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        double beta = w.cross(v).dot(n) / n.dot(n);
        double alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        bool projectionInTriangle = ((0 <= alpha) && (alpha <= 1) &&
                                     (0 <= beta)  && (beta  <= 1) &&
                                     (0 <= gamma) && (gamma <= 1));
        anyProjectionInTriangle = anyProjectionInTriangle || projectionInTriangle;
    }

    n.normalize();

    //we consider that the triangle goes through the cell if the proyection of the center
    //is inside the triangle AND the plane of the triangle intersects the cube of the cell

    return (anyProjectionInTriangle && (parallel || planeIntersects(n, query_point, cube)));
}


std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point, double cell_size)
{
    double cell_size_half = 0.5 * cell_size;

    std::vector<Eigen::Vector3d> points;
    points.emplace_back(query_point);
    points.emplace_back(query_point.x() - cell_size_half,
                        query_point.y() - cell_size_half,
                        query_point.z() - cell_size_half);
    points.emplace_back(query_point.x() - cell_size_half,
                        query_point.y() - cell_size_half,
                        query_point.z() + cell_size_half);
    points.emplace_back(query_point.x() - cell_size_half,
                        query_point.y() + cell_size_half,
                        query_point.z() - cell_size_half);
    points.emplace_back(query_point.x() - cell_size_half,
                        query_point.y() + cell_size_half,
                        query_point.z() + cell_size_half);
    points.emplace_back(query_point.x() + cell_size_half,
                        query_point.y() - cell_size_half,
                        query_point.z() - cell_size_half);
    points.emplace_back(query_point.x() + cell_size_half,
                        query_point.y() - cell_size_half,
                        query_point.z() + cell_size_half);
    points.emplace_back(query_point.x() + cell_size_half,
                        query_point.y() + cell_size_half,
                        query_point.z() - cell_size_half);
    points.emplace_back(query_point.x() + cell_size_half,
                        query_point.y() + cell_size_half,
                        query_point.z() + cell_size_half);
    return points;
}

bool planeIntersects(const Eigen::Vector3d &n,
                     const Eigen::Vector3d &plane_point,
                     const std::vector<Eigen::Vector3d> &cube)
{
    bool all_positive = true;
    bool all_negative = true;
    for (const Eigen::Vector3d &cube_point : cube)
    {
        double signo = n.dot(cube_point - plane_point);
        all_positive = all_positive && (signo > 0);
        all_negative = all_negative && (signo < 0);
    }
    return !all_positive && !all_negative;
}

//void fillStlGrid(openvdb::Vec3i empty_point, OccupancyGrid::Ptr &grid, int val, int empty)
//{
//    auto grid_accessor = grid->getAccessor();

//    std::queue<openvdb::Vec3i> q;
//    q.push(empty_point);

//    //env[x][y][1]=val;
//    openvdb::Coord grid_coord(empty_point.x(), empty_point.y(), 1);
//    grid_accessor.setValue(grid_coord, val);

//    while(!q.empty())
//    {
//        openvdb::Vec3i point = q.front();
//        q.pop();
//        if(point[0]+1<env.size()&&env[point[0]+1][point[1]][point[2]]==empty){ // x+1, y, z
//            env[point[0]+1][point[1]][point[2]]=val;
//            q.push(Eigen::Vector3i(point[0]+1,point[1],point[2]));
//        }
//        if(point[0]>0&&env[point[0]-1][point[1]][point[2]]==empty){ //x-1, y, z
//            env[point[0]-1][point[1]][point[2]]=val;
//            q.push(Eigen::Vector3i(point[0]-1,point[1],point[2]));
//        }
//        if(point[1]+1<env[0].size()&&env[point[0]][point[1]+1][point[2]]==empty){ //x, y+1, z
//            env[point[0]][point[1]+1][point[2]]=val;
//            q.push(Eigen::Vector3i(point[0],point[1]+1,point[2]));
//        }
//        if(point[1]>0&&env[point[0]][point[1]-1][point[2]]==empty){ //x, y-1, z
//            env[point[0]][point[1]-1][point[2]]=val;
//            q.push(Eigen::Vector3i(point[0],point[1]-1,point[2]));
//        }
//        if(point[2]+1<env[0][0].size()&&env[point[0]][point[1]][point[2]+1]==empty){ //x, y, z+1
//            env[point[0]][point[1]][point[2]+1]=val;
//            q.push(Eigen::Vector3i(point[0],point[1],point[2]+1));
//        }
//        if(point[2]>0&&env[point[0]][point[1]][point[2]-1]==empty){ //x, y, z-1
//            env[point[0]][point[1]][point[2]-1]=val;
//            q.push(Eigen::Vector3i(point[0],point[1],point[2]-1));
//        }
//    }
//}

//void clean(std::vector<std::vector<std::vector<int> > >& env){
//    std::stack<Eigen::Vector3i> st;
//    for(int col=0;col<env.size();col++){
//        for(int row=0;row<env[0].size();row++){
//            for(int height=0;height<env[0][0].size();height++){
//                if(env[col][row][height]==4){
//                    if((col<env.size()-1&&env[col+1][row][height]==3)||
//                            (row<env[0].size()-1&&env[col][row+1][height]==3)||
//                            (height<env[0][0].size()-1&&env[col][row][height+1]==3)||
//                            (col<env.size()-1&&row<env[0].size()-1&&env[col+1][row+1][height]==3
//                                &&env[col][row+1][height]==4
//                                &&env[col+1][row][height]==4))
//                    {
//                        env[col][row][height]=3;
//                    }else
//                    {
//                        env[col][row][height]=1;
//                    }

//                }

//            }
//        }
//    }
//}

} // namespace gaden





//void occupy(std::vector<std::vector<std::vector<int> > >& env,
//            std::vector<std::vector<std::vector<double> > > &points,
//            std::vector<std::vector<double> > &normals, int val){

//    //Let's occupy the enviroment!
//    for(int i= 0;i<points.size();i++){
//        //We try to find all the cells that some triangle goes through
//        int x1 = roundf((points[i][0][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
//        int y1 = roundf((points[i][0][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
//        int z1 = roundf((points[i][0][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
//        int x2 = roundf((points[i][1][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
//        int y2 = roundf((points[i][1][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
//        int z2 = roundf((points[i][1][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
//        int x3 = roundf((points[i][2][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
//        int y3 = roundf((points[i][2][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
//        int z3 = roundf((points[i][2][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));

//        int min_x = min_val(x1,x2,x3);
//        int min_y = min_val(y1,y2,y3);
//        int min_z = min_val(z1,z2,z3);

//        int max_x = max_val(x1,x2,x3);
//        int max_y = max_val(y1,y2,y3);
//        int max_z = max_val(z1,z2,z3);

//        bool xLimit = eq(std::fmod(max_val(points[i][0][0], points[i][1][0],points[i][2][0]) - env_min_x, cell_size), 0)
//                    ||eq(std::fmod(max_val(points[i][0][0], points[i][1][0],points[i][2][0]) - env_min_x, cell_size), cell_size);
//        bool yLimit = eq(std::fmod(max_val(points[i][0][1],points[i][1][1],points[i][2][1])-env_min_y, cell_size),0)||eq(std::fmod(max_val(points[i][0][1],points[i][1][1],points[i][2][1])-env_min_y, cell_size),cell_size);
//        bool zLimit = eq(std::fmod(max_val(points[i][0][2],points[i][1][2],points[i][2][2])-env_min_z, cell_size),0)||eq(std::fmod(max_val(points[i][0][2],points[i][1][2],points[i][2][2])-env_min_z, cell_size),cell_size);

//        if(x1<env[0].size()&&y1<env.size()&&z1<env[0][0].size()){
//            if((xLimit&&x1==max_x)||
//                (yLimit&&y1==max_y)||
//                (zLimit&&z1==max_z)
//                &&!env[y1][x1][z1]==1){
//                    env[y1][x1][z1]=(val==1?4:val);
//            }else{
//                env[y1][x1][z1] = val;
//            }
//        }

//        if(x2<env[0].size()&&y2<env.size()&&z2<env[0][0].size()){
//            if((xLimit&&x2==max_x)||
//                (yLimit&&y2==max_y)||
//                (zLimit&&z2==max_z)
//                &&!env[y2][x2][z2]==1){
//                    env[y2][x2][z2]=(val==1?4:val);
//            }else{
//                env[y2][x2][z2] = val;
//            }
//        }

//        if(x3<env[0].size()&&y3<env.size()&&z3<env[0][0].size()){
//            if((xLimit&&x3==max_x)||
//                (yLimit&&y3==max_y)||
//                (zLimit&&z3==max_z)
//                &&!env[y3][x3][z3]==1){
//                    env[y3][x3][z3]=(val==1?4:val);
//            }else{
//                env[y3][x3][z3] = val;
//            }
//        }

//        bool isParallel =parallel(normals[i]);

//        for (int row = min_x; row <= max_x && row < env[0].size(); row++)
//        {
//            for (int col = min_y; col <= max_y && col < env.size(); col++)
//            {
//                for (int height = min_z; height <= max_z && height < env[0][0].size(); height++)
//                {
//                    //check if the triangle goes through this cell
//                    if (pointInTriangle(Eigen::Vector3d(row * cell_size + env_min_x+cell_size/2,
//                                                        col * cell_size + env_min_y+cell_size/2,
//                                                        height * cell_size + env_min_z+cell_size/2),
//                                        Eigen::Vector3d(points[i][0][0], points[i][0][1], points[i][0][2]),
//                                        Eigen::Vector3d(points[i][1][0], points[i][1][1], points[i][1][2]),
//                                        Eigen::Vector3d(points[i][2][0], points[i][2][1], points[i][2][2]),
//                                        isParallel))
//                    {
//                        if((xLimit&&row==max_x)||
//                            (yLimit&&col==max_y)||
//                            (zLimit&&height==max_z)
//                            &&!env[col][row][height]==1){
//                                env[col][row][height]=(val==1?4:val);
//                        }else{
//                            env[col][row][height] = val;
//                        }
//                    }
//                }
//            }
//        }
//    }
//}
