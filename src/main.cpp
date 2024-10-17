#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Powerpack.h"
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>



#include <osqp.h>
#include <memory>


void printDataVector(const std::vector<double>& data_vector) {
    // 데이터 벡터의 내용을 출력
    ROS_INFO("Data Vector Contents:");
    for (size_t i = 0; i < data_vector.size(); ++i) {
        ROS_INFO("Element %zu: %f", i, data_vector[i]); // 각 원소를 출력
    }
}

int main(int argc, char* argv[]) {

  ////////////////yaml 파일 터미널에서 직접 받기 /////////////////
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>" << std::endl;
        return 1;
    }
    std::string filename = argv[1];
    YAML::Node config = YAML::LoadFile(filename);
    DatabaseConfig data(config);
//////////////////////////////////////////////////////

    ros::init(argc, argv, "Pos_neg_controller");
    ros::NodeHandle n;

///////////////parameter server에서 yaml 데이터 받기//////////////////
    // std::string yaml_file;
    // if (!n.getParam("yaml_file", yaml_file)) {
    //     ROS_ERROR("Could not find parameter 'yaml_file'");
    //     return 1;
    // }

    // YAML::Node config = YAML::LoadFile(yaml_file);
    // DatabaseConfig data(config);

 ///////////////////////////////////////////////////////////////////   
 
    Powerpack powerpack(data.get_n_pos_channel(), data.get_n_neg_Channel(), data.get_MPC_parameters());

    ros::Subscriber sen_sub = n.subscribe<std_msgs::Float32MultiArray>("sen_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        for (const auto& value : data->data) {
            data_vector.push_back(static_cast<double>(value));
        }
        powerpack.update_sensor(data_vector);
        std::cout << "sensor data" << std::endl;
        printDataVector(powerpack.get_all_sensor_data());
        
      } 
    );

    ros::Subscriber ref_sub = n.subscribe<std_msgs::Float32MultiArray>("ref_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        for (const auto& value : data->data) {
            data_vector.push_back(static_cast<double>(value));
        }
        powerpack.update_reference(data_vector);
         std::cout << "sensor data" << std::endl;
        printDataVector(powerpack.get_all_reference_data());
      } 
    );
    ros::Publisher mpc_pwm_pub = n.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 10);
    ros::Rate loop_rate(100); // 100 Hz

    while (ros::ok())
  {

    if (powerpack.get_all_sensor_data()[0] == 0) {
        std::cout << powerpack.get_all_sensor_data()[0] << std::endl;
        ROS_INFO("Still in initial state, waiting for updates...");
    }
    else {
        powerpack.run();

        std::vector<unsigned int> data = powerpack.get_control_signal();
        std_msgs::UInt16MultiArray msg;
        msg.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
        msg.data[i] = data[i];
        }
        
        mpc_pwm_pub.publish(msg);

    }
        ros::spinOnce();
        loop_rate.sleep();
  }

  return 0;
}


