#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <osqp.h>
#include <memory>

#include "Powerpack.h"
// #include "DatabaseConfig.h"


int main(int argc, char* argv[]) {

  //////////////yaml 파일 터미널에서 직접 받기 /////////////////
    if (argc != 2) {
       std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>" << std::endl;
       return 1;
    }
    std::string filename = argv[1];
    YAML::Node config = YAML::LoadFile(filename);
    std::shared_ptr<DatabaseConfig> databaseconfig = std::make_shared<DatabaseConfig>(config);
//////////////////////////////////////////////////////

    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;

///////////////parameter server에서 yaml 데이터 받기//////////////////
    //  std::string yaml_file;
    //  if (!n.getParam("yaml_file", yaml_file)) {
    //      ROS_ERROR("Could not find parameter 'yaml_file'");
    //      return 1;
    //  }

    //  YAML::Node config = YAML::LoadFile(yaml_file);
    //  DatabaseConfig databaseconfig(config);
 ///////////////////////////////////////////////////////////////////   
 
    Powerpack powerpack(nh, databaseconfig);

//     ros::Subscriber sen_sub = n.subscribe<std_msgs::Float32MultiArray>("sen_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
//         std::vector<double> data_vector;
//         data_vector.reserve(data->data.size());

//         for (const auto& value : data->data) {
//             data_vector.push_back(static_cast<double>(value));
//         }
//         powerpack.update_sensor(data_vector);
        
//       } 
//     );

//     ros::Subscriber ref_sub = n.subscribe<std_msgs::Float32MultiArray>("ref_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
//         std::vector<double> data_vector;
//         data_vector.reserve(data->data.size());

//         for (const auto& value : data->data) {
//             data_vector.push_back(static_cast<double>(value));
//         }
//         powerpack.update_reference(data_vector);
//         // printDataVector(data_vector);
//       } 
//     );

//     ros::Publisher mpc_pwm_pub = n.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 10);
    ros::Rate loop_rate(100); // 100 Hz
    // ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok())
  {
      // for(const auto& data : powerpack.get_reference_data()) {
      //   ROS_INFO("Reference data : %f", data);
      // }
      // ROS_INFO("=======================");

//     if (powerpack.get_all_sensor_data()[0] == 0) {
//         std::cout << powerpack.get_all_sensor_data()[0] << std::endl;
//         ROS_INFO("Still in initial state, waiting for sensor updates...");
//     }
//     else {
//         system("clear");
//         ROS_INFO("======sensor======");
//         printDataVector(powerpack.get_all_sensor_data());
//         ROS_INFO("==================");
//         ROS_INFO("======reference======");
//         printDataVector(powerpack.get_all_reference_data());
//         ROS_INFO("==================");
//         powerpack.run();

//         std::vector<unsigned int> data = powerpack.get_control_signal();
//         std_msgs::UInt16MultiArray msg;
//         msg.data.resize(data.size());
//         for (size_t i = 0; i < data.size(); ++i) {
//         msg.data[i] = data[i];
//         }
//         mpc_pwm_pub.publish(msg);

//         ROS_INFO("======MPC Control published======");
//         for (const auto& tmp_data : data) {
//             ROS_INFO("Element : %u", tmp_data);
//         }
//             ROS_INFO("==================");
//     }
        ros::spinOnce();
        loop_rate.sleep();
  }
  return 0;
}


