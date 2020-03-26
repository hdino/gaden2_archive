#include <gaden_preprocessing/math_helper.h>
#include <gaden_preprocessing/occupancy_grid.h>
#include <gaden_preprocessing/stl_data.h>

#include <array>
#include <algorithm>
#include <cmath>

#include <iostream>

namespace gaden {

struct MinMaxVector
{
    openvdb::Vec3i min;
    openvdb::Vec3i max;
};

template <typename TContainer>
MinMaxVector getElementWiseMinMax(const TContainer &input)
{
    auto min_max_x = std::minmax_element(input.begin(), input.end(),
                                         [](const auto &a, const auto &b) { return a.x() < b.x(); });
    auto min_max_y = std::minmax_element(input.begin(), input.end(),
                                         [](const auto &a, const auto &b) { return a.y() < b.y(); });
    auto min_max_z = std::minmax_element(input.begin(), input.end(),
                                         [](const auto &a, const auto &b) { return a.z() < b.z(); });
    MinMaxVector result;
    result.min.x() = min_max_x.first->x();
    result.min.y() = min_max_y.first->y();
    result.min.z() = min_max_z.first->z();

    result.max.x() = min_max_x.second->x();
    result.max.y() = min_max_y.second->y();
    result.max.z() = min_max_z.second->z();

    return result;
}

StlData::StlData(std::vector<StlFacet> &&facets)
    : facets_(std::move(facets))
{
    //
}

bool StlData::hasData() const
{
    return !facets_.empty();
}

void StlData::addToOccupancyGrid(OccupancyGrid::Ptr &grid, double cell_size) const
{
    auto grid_accessor = grid->getAccessor();

    for (const StlFacet &facet : facets_)
    {
        auto vertex_cell_coordinates = getCellCoordinates(facet.vertices, cell_size);

        auto cell = getElementWiseMinMax(vertex_cell_coordinates);

//        auto max_x = std::max({facet.vertex[0][0], facet.vertex[1][0], facet.vertex[2][0]});
//        auto max_y = std::max({facet.vertex[0][1], facet.vertex[1][1], facet.vertex[2][1]});
//        auto max_z = std::max({facet.vertex[0][2], facet.vertex[1][2], facet.vertex[2][2]});

//        auto max_x_grid_deviation = std::fmod(max_x, cell_size);
//        auto max_y_grid_deviation = std::fmod(max_y, cell_size);
//        auto max_z_grid_deviation = std::fmod(max_z, cell_size);

//        bool x_limit = Tolerant(max_x_grid_deviation) == 0 || Tolerant(max_x_grid_deviation) == cell_size;
//        bool y_limit = Tolerant(max_y_grid_deviation) == 0 || Tolerant(max_y_grid_deviation) == cell_size;
//        bool z_limit = Tolerant(max_z_grid_deviation) == 0 || Tolerant(max_z_grid_deviation) == cell_size;

//        openvdb::Coord grid_coord(vertex_cell_coordinates[0]);

//        if (    (x_limit && vertex_cell_coordinates[0].x() == min_max_cell_x.max)
//             || (y_limit && vertex_cell_coordinates[0].y() == min_max_cell_y.max)
//             || (z_limit && vertex_cell_coordinates[0].z() == min_max_cell_z.max)
//             && !grid_accessor.getValue(grid_coord) == toUnderlying(Occupancy::Occupied))
//        {
//            grid_accessor.setValue(grid_coord) = 4; // TODO This value should have a name / assert that it's gone after preprocessing
//        }
//        else
//            grid_accessor.setValue(grid_coord) = toUnderlying(Occupancy::Occupied);


    }
}

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








//void findDimensions(const std::string &filename, rclcpp::Logger &logger)
//{
//    std::string line;
//    std::getline(file_stream, line);
//    int i =0;
//    while (line.find("endsolid")==std::string::npos)
//        {
//            while (std::getline(file_stream, line) && line.find("outer loop") == std::string::npos);

//            for(int j=0;j<3;j++){
//                double x, y, z;
//                std::getline(file_stream, line);
//                size_t pos = line.find("vertex ");
//                line.erase(0, pos + 7);
//                std::stringstream ss(line);
//                double aux;
//                ss >> std::skipws >>  aux;
//                x = roundf(aux * roundFactor) / roundFactor;
//                ss >> std::skipws >>  aux;
//                y = roundf(aux * roundFactor) / roundFactor;
//                ss >> std::skipws >>  aux;
//                z = roundf(aux * roundFactor) / roundFactor;
//                env_max_x = env_max_x>=x?env_max_x:x;
//                env_max_y = env_max_y>=y?env_max_y:y;
//                env_max_z = env_max_z>=z?env_max_z:z;
//                env_min_x = env_min_x<=x?env_min_x:x;
//                env_min_y = env_min_y<=y?env_min_y:y;
//                env_min_z = env_min_z<=z?env_min_z:z;
//            }
//            i++;
//            //skipping three lines here makes checking for the end of the file more convenient
//            std::getline(file_stream, line);
//            std::getline(file_stream, line);
//            while(std::getline(file_stream, line)&&line.length()==0);
//    }
//}

//void parse(std::string filename, std::vector<std::vector<std::vector<int> > >& env, int val){
//    //first, we count how many triangles there are (we need to do this before reading the data
//    // to create a vector of the right size)
//    std::ifstream countfile(filename.c_str());
//    std::string line;
//    int count = 0;

//    while (std::getline(countfile, line)){
//        if(line.find("facet normal") != std::string::npos){
//            count++;
//        }
//    }
//    //each points[i] contains one the three vertices of triangle i
//    std::vector<std::vector<std::vector<double> > >points(count, std::vector<std::vector<double> > (3, std::vector<double>(3)));
//    std::vector<std::vector<double> > normals(count, std::vector<double>(3));
//    //let's read the data
//    std::ifstream infile(filename.c_str());
//    std::getline(infile, line);
//    int i =0;
//    while (line.find("endsolid")==std::string::npos)
//        {
//            while (line.find("facet normal") == std::string::npos){std::getline(infile, line);}
//            size_t pos = line.find("facet");
//            line.erase(0, pos + 12);
//            double aux;
//            std::stringstream ss(line);
//            ss >> std::skipws >>  aux;
//            normals[i][0] = roundf(aux * roundFactor) / roundFactor;
//            ss >> std::skipws >>  aux;
//            normals[i][1] = roundf(aux * roundFactor) / roundFactor;
//            ss >> std::skipws >>  aux;
//            normals[i][2] = roundf(aux * roundFactor) / roundFactor;
//            std::getline(infile, line);

//            for(int j=0;j<3;j++){
//                std::getline(infile, line);
//                size_t pos = line.find("vertex ");
//                line.erase(0, pos + 7);
//                std::stringstream ss(line);
//                ss >> std::skipws >>  aux;
//                points[i][j][0] = roundf(aux * roundFactor) / roundFactor;
//                ss >> std::skipws >>  aux;
//                points[i][j][1] = roundf(aux * roundFactor) / roundFactor;
//                ss >> std::skipws >>  aux;
//                points[i][j][2] = roundf(aux * roundFactor) / roundFactor;
//            }
//            i++;
//            //skipping lines here makes checking for the end of the file more convenient
//            std::getline(infile, line);
//            std::getline(infile, line);
//            while(std::getline(infile, line)&&line.length()==0);
//    }
//    //OK, we have read the data, let's do something with it
//    occupy(env, points, normals, val);

//}







//std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point){
//    std::vector<Eigen::Vector3d> points;
//    points.push_back(query_point);
//    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
//                                            query_point.y()-cell_size/2,
//                                            query_point.z()-cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
//                                            query_point.y()-cell_size/2,
//                                            query_point.z()+cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
//                                            query_point.y()+cell_size/2,
//                                            query_point.z()-cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
//                                            query_point.y()+cell_size/2,
//                                            query_point.z()+cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
//                                            query_point.y()-cell_size/2,
//                                            query_point.z()-cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
//                                            query_point.y()-cell_size/2,
//                                            query_point.z()+cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
//                                            query_point.y()+cell_size/2,
//                                            query_point.z()-cell_size/2));
//    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
//                                            query_point.y()+cell_size/2,
//                                            query_point.z()+cell_size/2));
//    return points;
//}

//bool planeIntersects(Eigen::Vector3d& n, const Eigen::Vector3d& planePoint,const std::vector<Eigen::Vector3d>& cube){
//    bool allPositive = true;
//    bool allNegative = true;
//    for (int i=0;i<cube.size();i++){
//        double signo = n.dot(cube[i]-planePoint);
//        allPositive = allPositive&&(signo>0);
//        allNegative = allNegative&&(signo<0);
//    }
//    return !allPositive&&!allNegative;
//}

//bool pointInTriangle(const Eigen::Vector3d& query_point,
//                     const Eigen::Vector3d& triangle_vertex_0,
//                     const Eigen::Vector3d& triangle_vertex_1,
//                     const Eigen::Vector3d& triangle_vertex_2,
//                     bool parallel)
//{
//    // u=P2−P1
//    Eigen::Vector3d u = triangle_vertex_1 - triangle_vertex_0;
//    // v=P3−P1
//    Eigen::Vector3d v = triangle_vertex_2 - triangle_vertex_0;
//    // n=u×v
//    Eigen::Vector3d n = u.cross(v);
//    bool anyProyectionInTriangle=false;
//    std::vector<Eigen::Vector3d> cube= cubePoints(query_point);
//    for(const Eigen::Vector3d &vec : cube){
//        // w=P−P1
//        Eigen::Vector3d w = vec - triangle_vertex_0;
//        // Barycentric coordinates of the projection P′of P onto T:
//        // γ=[(u×w)⋅n]/n²
//        double gamma = u.cross(w).dot(n) / n.dot(n);
//        // β=[(w×v)⋅n]/n²
//        double beta = w.cross(v).dot(n) / n.dot(n);
//        double alpha = 1 - gamma - beta;
//        // The point P′ lies inside T if:
//        bool proyectionInTriangle= ((0 <= alpha) && (alpha <= 1) &&
//                (0 <= beta)  && (beta  <= 1) &&
//                (0 <= gamma) && (gamma <= 1));
//        anyProyectionInTriangle=anyProyectionInTriangle||proyectionInTriangle;
//    }

//    n.normalize();

//    //we consider that the triangle goes through the cell if the proyection of the center
//    //is inside the triangle AND the plane of the triangle intersects the cube of the cell

//    return (anyProyectionInTriangle && (parallel||planeIntersects(n, query_point,cube)));
//}

//bool parallel (std::vector<double> &vec){
//    return (eq(vec[1],0)
//                &&eq(vec[2],0))||
//           (eq(vec[0],0)
//                &&eq(vec[2],0))||
//           (eq(vec[0],0)
//                &&eq(vec[1],0));
//}
