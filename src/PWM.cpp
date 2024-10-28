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

        publisher = nh.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 100);
}

PWM::~PWM() {
}

void PWM::update_pwm(std::vector<double> data) {
        std_msgs::UInt16MultiArray pwm_data;
        pwm_data.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            pwm_data.data[i] = static_cast<uint16_t>(10 * data[i]);
        }
        publisher.publish(pwm_data);
}