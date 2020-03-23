#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
//    ros::init(argc, argv, "preprocessing");
//    ros::NodeHandle nh;
//    ros::NodeHandle private_nh("~");
    //Init
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("environment", node_options);

    //int numModels;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("preprocessing_done",5,true);

    private_nh.param<double>("cell_size", cell_size, 1); //size of the cells

    roundFactor=1000.0/cell_size;
    //stl file with the model of the outlets
    std::string outlet; int numOutletModels;

    //path to the csv file where we want to write the occupancy map
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    //--------------------------

        //OCCUPANCY

    //--------------------------

    private_nh.param<int>("number_of_models", numModels, 2); // number of CAD models

    std::vector<std::string> CADfiles;
    for(int i = 0; i< numModels; i++){
        std::string paramName = boost::str( boost::format("model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        CADfiles.push_back(filename.c_str());
    }

    for (int i = 0; i < CADfiles.size(); i++)
    {
        findDimensions(CADfiles[i]);
    }
    std::vector<std::vector<std::vector<int> > > env(ceil((env_max_y-env_min_y)*(roundFactor)/(cell_size*(roundFactor))),
                                                    std::vector<std::vector<int> >(ceil((env_max_x - env_min_x)*(roundFactor)/(cell_size*(roundFactor))),
                                                                                    std::vector<int>(ceil((env_max_z - env_min_z)*(roundFactor)/(cell_size*(roundFactor))), 0)));

    for (int i = 0; i < numModels; i++)
    {
        parse(CADfiles[i], env, 1);
    }


    double empty_point_x;
    private_nh.param<double>("empty_point_x", empty_point_x, 1);
    double empty_point_y;
    private_nh.param<double>("empty_point_y", empty_point_y, 1);
    double empty_point_z;
    private_nh.param<double>("empty_point_z", empty_point_z, 1);

    std::cout<<"Filling...\n";
    fill((empty_point_x-env_min_x)/cell_size,
        (empty_point_y-env_min_y)/cell_size,
        (empty_point_z-env_min_z)/cell_size,
        env, 3, 0);
    clean(env);


    printEnv(boost::str(boost::format("%s/occupancy.pgm") % output.c_str()), env, 10);

    //--------------------------

        //OUTLETS

    //--------------------------

    private_nh.param<int>("number_of_outlet_models", numOutletModels, 1); // number of CAD models

    std::vector<std::string> outletFiles;
    for(int i = 0; i< numOutletModels; i++){
        std::string paramName = boost::str( boost::format("outlets_model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        outletFiles.push_back(filename.c_str());
    }

    for (int i=0;i<numOutletModels; i++){
        parse(outletFiles[i], env, 2);
    }

    fill((empty_point_x-env_min_x)/cell_size,
        (empty_point_y-env_min_y)/cell_size,
        (empty_point_z-env_min_z)/cell_size,
        env, 5, 3);

    //output - path, occupancy vector, scale
    printEnv(boost::str(boost::format("%s/OccupancyGrid3D.csv") % output.c_str()), env, 1);
    printYaml(output);

    //-------------------------

        //WIND

    //-------------------------

    bool uniformWind;
    private_nh.param<bool>("uniformWind", uniformWind, false);

    //path to the point cloud files with the wind data
    std::string windFileName;
    private_nh.param<std::string>("wind_files", windFileName, "");
    int idx = 0;

    if(uniformWind){

        //let's parse the file
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<std::vector<std::vector<double> > > U(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        std::vector<std::vector<std::vector<double> > > V(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        std::vector<std::vector<std::vector<double> > > W(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        while(std::getline(infile, line)){
            std::vector<double> v;
            for (int i = 0; i < 3; i++)
            {
                size_t pos = line.find(",");
                v.push_back(atof(line.substr(0, pos).c_str()));
                line.erase(0, pos + 1);
            }

            for(int i = 0; i< env[0].size();i++){
                for(int j = 0; j< env.size();j++){
                    for(int k = 0; k< env[0][0].size();k++){
                        if(env[j][i][k]==5){
                            U[i][j][k] = v[0];
                            V[i][j][k] = v[1];
                            W[i][j][k] = v[2];
                        }
                    }
                }
            }
            printWind(U,V,W, boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }else{
        while (FILE *file = fopen(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), "r"))
        {
            fclose(file);
            openFoam_to_gaden(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), env);
            idx++;
        }
    }



    ROS_INFO("Preprocessing done");
    std_msgs::Bool b;
    b.data=true;
    ros::Rate r(0.1);
    while(ros::ok()){
        pub.publish(b);
        r.sleep();
    }
}
