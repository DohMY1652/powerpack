#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <memory>
#include <vector>

#include "DatabaseConfig.h"

class Sensor {
   public:
    Sensor(ros::NodeHandle& nh,
           std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Sensor();

    void subscriber_callback(
        const std_msgs::Float32MultiArray::ConstPtr& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        assert(data->data.size() == n_pump_channel + n_macro_channel +
                                        n_pos_channel + n_neg_channel);

        if (is_initialized) {
            std::vector<float> raw_data = data->data;
            data_vector.push_back(static_cast<double>(raw_data[0] - offset[0]) *
                                      pos_gain +
                                  atm_offset + 300);
            data_vector.push_back(static_cast<double>(raw_data[1] - offset[1]) *
                                      neg_gain +
                                  atm_offset - 70);
            data_vector.push_back(static_cast<double>(raw_data[2] - offset[2]) *
                                      pos_gain +
                                  atm_offset + 700);
            for (int index = (n_pump_channel + n_macro_channel);
                 index < (n_pump_channel + n_macro_channel + n_pos_channel);
                 ++index) {
                data_vector.push_back(
                    static_cast<double>(raw_data[index] - offset[index]) *
                        pos_gain +
                    atm_offset);
            }
            for (int index = (n_pump_channel + n_macro_channel + n_pos_channel);
                 index < (n_pump_channel + n_macro_channel + n_pos_channel +
                          n_neg_channel);
                 ++index) {
                data_vector.push_back(
                    static_cast<double>(raw_data[index] - offset[index]) *
                        neg_gain +
                    atm_offset);
            }
            update(data_vector);
        } else {
            std::vector<double> data_vector;
            data_vector.reserve(data->data.size());
            for (const auto& value : data->data) {
                data_vector.push_back(std::move((double)(value)));
            }
            initialize(data_vector);
        }
        // ROS_INFO("============");
        // for (const auto& value : data_vector) {
        //     ROS_INFO("Sensor data : %f", value);
        // }
    }

    void initialize(std::vector<double> data_vector);
    void update(const std::vector<double> _data);

    std::vector<double> get_data() const;

   private:
    ros::Subscriber subscriber;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    bool is_initialized;

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

    std::vector<double> offset;
    std::vector<double> data;
};

#endif  // SENSOR_H
