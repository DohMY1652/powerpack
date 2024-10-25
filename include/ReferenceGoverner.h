#ifndef REFERENCEGOVERNER_H
#define REFERENCEGOVERNER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <memory>
#include <vector>

#include "DatabaseConfig.h"

class ReferenceGoverner {
   public:
    ReferenceGoverner(ros::NodeHandle& nh,
                      std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~ReferenceGoverner();

    void subscriber_callback(
        const std_msgs::Float32MultiArray::ConstPtr& data) {
        std::vector<double> data_vector;
        assert(data->data.size() == n_channel);
        data_vector.reserve(data->data.size());
        for (const auto& value : data->data) {
            data_vector.push_back(std::move((double)(value)));
        }
        update(data_vector);
        // ROS_INFO("============");
        // for (const auto& value : data_vector) {
        //     ROS_INFO("Reference data : %f", value);
        // }
    }

    void update(const std::vector<double> _data);

    std::vector<double> get_data() const;

   private:
    ros::Subscriber subscriber;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    int frequency;
    int n_channel;
    std::vector<double> data;
};

#endif  // REFERENCE_GOVERNER_H