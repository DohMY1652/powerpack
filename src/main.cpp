#include <osqp.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>

#include "Powerpack.h"
// #include "DatabaseConfig.h"

bool is_initialized = false;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;

    //////////////yaml 파일 터미널에서 직접 받기 /////////////////
    // if (argc != 2) {
    //    std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>"
    //              << std::endl;
    //    return 1;
    // }
    // std::string filename = argv[1];
    // YAML::Node config = YAML::LoadFile(filename);
    // std::shared_ptr<DatabaseConfig> databaseconfig =
    //    std::make_shared<DatabaseConfig>(config);
    //////////////////////////////////////////////////////

    ///////////////parameter server에서 yaml 데이터 받기//////////////////
      std::string yaml_file;
      if (!nh.getParam("yaml_file", yaml_file)) {
          ROS_ERROR("Could not find parameter 'yaml_file'");
          return 1;
      }

      YAML::Node config = YAML::LoadFile(yaml_file);
      std::shared_ptr<DatabaseConfig> databaseconfig = std::make_shared<DatabaseConfig>(config);
    
    ///////////////////////////////////////////////////////////////////
    auto powerpack = std::make_unique<Powerpack>(nh, databaseconfig);

    // Powerpack powerpack(nh, databaseconfig);

    ros::Rate loop_rate(50);  // 50 Hz


    // ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        if (powerpack->get_sensor_data()[0] == 0) {
            ROS_INFO("Still in initial state, waiting for sensor updates...");
            
        } else {
            if (is_initialized) {
            // std::vector<double> data = powerpack->get_reference_data();
            // ROS_INFO("==============");
            // for (const auto& tmp : data) {
            //     ROS_INFO("%f", tmp);
            // }
            powerpack->run();
            }
            else {
                // ROS_INFO("Sleep 10 seconds");
                // ros::Duration(10.0).sleep();
                is_initialized = true;
            }


        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
