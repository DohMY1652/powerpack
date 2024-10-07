#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Powerpack.h"
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char* argv[]) {
    // if (argc != 2) {
    //     std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>" << std::endl;
    //     return 1;
    // }
    // std::string filename = argv[1];
    // YAML::Node config = YAML::LoadFile(filename);
    // DatabaseConfig data(config);
    
    ros::init(argc, argv, "Pos_neg_controller");
    ros::NodeHandle n;

    std::string yaml_file;
    if (!n.getParam("yaml_file", yaml_file)) {
        ROS_ERROR("Could not find parameter 'yaml_file'");
        return 1;
    }

    YAML::Node config = YAML::LoadFile(yaml_file);
    DatabaseConfig data(config);
    Powerpack powerpack(data.get_n_pos_channel(), data.get_n_neg_Channel(), data.get_pos_pid_gains(), data.get_neg_pid_gains());

    ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("sen_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        for (const auto& value : data->data) {
            data_vector.push_back(static_cast<double>(value));
        }
        powerpack.update_sensor(data_vector);
      } 
    );
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 10);
    ros::Rate loop_rate(100); // 100 Hz

    while (ros::ok())
  {


    powerpack.run();

    std::vector<unsigned int> data = powerpack.get_control_signal();
    std_msgs::UInt16MultiArray msg;
    msg.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
    msg.data[i] = data[i];
    }
    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


