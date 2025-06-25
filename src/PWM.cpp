// PWM.cpp
#include <iostream>

#include "PWM.h"

PWM::PWM(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig> &databaseconfig)
 :  databaseconfig(databaseconfig),
    n_channel(databaseconfig->get_n_pid_channel() 
            + 3 * (databaseconfig->get_n_pos_channel()
            + databaseconfig->get_n_neg_channel())) {

        std::vector<double> parameters = databaseconfig->get_sensor_parameters();
        frequency = (int)parameters[0];
        pid_pos_index = (int)parameters[1];
        pid_neg_index = (int)parameters[2];
        data.resize(n_channel);
        std::vector<bool> system_parameters = databaseconfig->get_system_parameters();
        printing = system_parameters[0];
        operating = system_parameters[1];


        publisher = nh.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 1);
        raw_publisher = nh.advertise<std_msgs::UInt16MultiArray>("raw_mpc_pwm", 1);
}

PWM::~PWM() {
}

void PWM::update_pwm(std::vector<double> data) {
        std_msgs::UInt16MultiArray pwm_data;
        std_msgs::UInt16MultiArray raw_pwm_data;
        pwm_data.data.resize(data.size());
        raw_pwm_data.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
              raw_pwm_data.data[i] = static_cast<uint16_t>(10 * data[i]);
           } 
        
        if(operating) {
           for (size_t i = 0; i < data.size(); ++i) {
              pwm_data.data[i] = static_cast<uint16_t>(10 * data[i]);
           } 
        }
        else {
           for (size_t i = 0; i < data.size(); ++i) {
              pwm_data.data[i] = static_cast<uint16_t>(0);
           }  
        }
        raw_publisher.publish(raw_pwm_data);
        publisher.publish(pwm_data);
        if(printing) {
         //   for (const double& value : data) {
         //        std::cout << value << " ";
         //   }
         //   std::cout << std::endl;
        }
}