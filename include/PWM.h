// PWM.h
#ifndef PWM_H
#define PWM_H

#include <iostream>
#include <vector>
#include <memory>
#include <std_msgs/UInt16MultiArray.h>

#include "DatabaseConfig.h"

class PWM {
public:
    PWM(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig> &databaseconfig);
    ~PWM();

    void update_pwm(std::vector<double> data);

private:
    std::shared_ptr<DatabaseConfig> &databaseconfig;

    ros::Publisher publisher;

    int n_channel;
    int frequency;
    int pid_pos_index;
    int pid_neg_index;

    std::vector<unsigned int> data;
};

#endif // PWM_H