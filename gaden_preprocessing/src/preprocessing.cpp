#include <gaden_common/ros_parameters.h>
#include <gaden_preprocessing/preprocessing.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace gaden {

template <typename T>
openvdb::math::Vec3<T> getOpenVdbVec3FromYaml(const YAML::Node &sequence)
{
    openvdb::math::Vec3<T> vector;
    vector.x() = sequence[0].as<T>();
    vector.y() = sequence[1].as<T>();
    vector.z() = sequence[2].as<T>();
    return vector;
}

PreprocessingConfig loadPreprocessingConfig(std::shared_ptr<rclcpp::Node> &ros_node)
{
    PreprocessingConfig config;

    std::string base_path;
    if (!getNodeParameter(ros_node, "base_path", base_path))
        return config;

    YAML::Node yaml_config = YAML::LoadFile(base_path + "config.yaml");

    // common part
    YAML::Node yaml_common = yaml_config["common"];

    for (const YAML::Node &item : yaml_common["environment"])
    {
        CadModel cad_model = getCadModelFromYaml(item, base_path);
        config.environment_cad_models.push_back(cad_model);
    }

    for (const YAML::Node &item : yaml_common["outlets"])
    {
        CadModel cad_model = getCadModelFromYaml(item, base_path);
        config.outlet_cad_models.push_back(cad_model);
    }

    // preprocessing part
    YAML::Node preprocessing = yaml_config["preprocessing"];
    std::cout << preprocessing << std::endl;

    config.cell_size = preprocessing["cell_size"].as<double>(); // TODO Default value 1.0?
    config.output_path = base_path + preprocessing["output_path"].as<std::string>();
    config.empty_point = getOpenVdbVec3FromYaml<double>(preprocessing["empty_point"]);

    return config;
}

} // namespace gaden


//void printEnv(std::string filename, std::vector<std::vector<std::vector<int> > > env, int scale)
//{
//    std::ofstream outfile(filename.c_str());
//    if (filename.find(".pgm") != std::string::npos)
//    {
//        outfile << "P2\n"
//                << scale *  env[0].size() << " " << scale * env.size() << "\n" <<"1\n";
//        //things are repeated to scale them up (the image is too small!)
//        for (int row = env.size()-1; row >= 0; row--)
//        {
//            for (int j = 0; j < scale; j++)
//            {
//                for (int col = 0; col <env[0].size() ; col++)
//                {
//                    for (int i = 0; i < scale; i++)
//                    {
//                        outfile << (env[row][col][0] == 3 ? 1 : 0) << " ";
//                    }
//                }
//                outfile << "\n";
//            }
//        }
//    }
//    else
//    {
//        outfile <<  "#env_min(m) " << env_min_x << " " << env_min_y << " " << env_min_z << "\n";
//        outfile <<  "#env_max(m) " << env_max_x << " " << env_max_y << " " << env_max_z << "\n";
//        outfile <<  "#num_cells " << env[0].size() << " " << env.size() << " " << env[0][0].size() << "\n";
//        outfile <<  "#cell_size(m) " << cell_size << "\n";
//        //things are repeated to scale them up (the image is too small!)
//        for (int height = 0; height < env[0][0].size(); height++)
//        {
//            for (int col = 0; col <env[0].size(); col++)
//            {
//                for (int j = 0; j < scale; j++)
//                {
//                    for (int row = 0; row <env.size(); row++)
//                    {
//                        for (int i = 0; i < scale; i++)
//                        {
//                            outfile << (env[row][col][height]==0?1:
//                                            (env[row][col][height]==3?2:
//                                            (env[row][col][height]==5?0:
//                                                env[row][col][height])))
//                                    << " ";
//                        }
//                    }
//                    outfile << "\n";
//                }
//            }
//            outfile << ";\n";
//        }
//    }
//}
//void printWind(std::vector<std::vector<std::vector<double> > > U,
//                std::vector<std::vector<std::vector<double> > > V,
//                std::vector<std::vector<std::vector<double> > > W, std::string filename){
    
//    std::ofstream fileU(boost::str(boost::format("%s_U") % filename).c_str());
//    std::ofstream fileV(boost::str(boost::format("%s_V") % filename).c_str());
//    std::ofstream fileW(boost::str(boost::format("%s_W") % filename).c_str());
//    for (int height = 0; height < U[0][0].size(); height++)
//    {
//        for (int col = 0; col < U.size(); col++)
//        {
//            for (int row = 0; row < U[0].size(); row++)
//            {
//                fileU << U[col][row][height] << " ";
//                fileV << V[col][row][height] << " ";
//                fileW << W[col][row][height] << " ";
//            }
//            fileU << "\n";
//            fileV << "\n";
//            fileW << "\n";
//        }
//        fileU << ";\n";
//        fileV << ";\n";
//        fileW << ";\n";
//    }
//}

//void printYaml(std::string output){
//    std::ofstream yaml(boost::str(boost::format("%s/occupancy.yaml") % output.c_str()));
//    yaml << "image: occupancy.pgm\n"
//        << "resolution: " << cell_size/10
//        << "\norigin: [" << env_min_x << ", " << env_min_y << ", " << env_min_z << "]\n"
//        << "occupied_thresh: 0.9\n"
//        << "free_thresh: 0.1\n"
//        << "negate: 0";
//}






//void openFoam_to_gaden(std::string filename, std::vector<std::vector<std::vector<int> > >& env)
//{

//	//let's parse the file
//	std::ifstream infile(filename.c_str());
//	std::string line;

//	//ignore the first line (column names)
//	std::getline(infile, line);
//    std::vector<std::vector<std::vector<double> > > U(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
//    std::vector<std::vector<std::vector<double> > > V(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
//    std::vector<std::vector<std::vector<double> > > W(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
//    std::vector<double> v(6);
//	int x_idx = 0;
//	int y_idx = 0;
//	int z_idx = 0;
//	while (std::getline(infile, line))
//	{
//		if (line.length()!=0)
//		{
//			for (int i = 0; i < 6; i++)
//			{
//				size_t pos = line.find(",");
//				v[i] = atof(line.substr(0, pos).c_str());
//				line.erase(0, pos + 1);
//			}
//			//assign each of the points we have information about to the nearest cell
//			x_idx = roundf((v[3] - env_min_x) / cell_size*roundFactor)/roundFactor;
//			y_idx = roundf((v[4] - env_min_y) / cell_size*roundFactor)/roundFactor;
//			z_idx = roundf((v[5] - env_min_z) / cell_size*roundFactor)/roundFactor;
//			U[x_idx][y_idx][z_idx] = v[0];
//			V[x_idx][y_idx][z_idx] = v[1];
//			W[x_idx][y_idx][z_idx] = v[2];
//		}
//	}
//    printWind(U,V,W,filename);
//}
