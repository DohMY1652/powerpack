#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "DatabaseConfig.h"

class Sensor {
public:
    Sensor(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig> &databaseconfig);
    ~Sensor();


    void subscriber_callback(const std_msgs::Float32MultiArray::ConstPtr& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());
        for (const auto& value : data->data) {
            data_vector.push_back(std::move((double)(value)));
        }
        update(data_vector);
    }

    void update(const std::vector<double> _data);
    
    std::vector<double> get_data() const;



private:
    ros::Subscriber subscriber;
    std::shared_ptr<DatabaseConfig> &databaseconfig;

    int frequency;
    int n_pump_channel;
    int n_macro_channel;
    int n_pos_channel;
    int n_neg_channel;

    double pos_offset;
    double pos_gain;
    double neg_offset;
    double neg_gain;
    double atm_offset;

    int pressure_pos_index;
    int pressure_neg_index;
    int pressure_macro_index;


    std::vector<double> data;
};

#endif //SENSOR_H

