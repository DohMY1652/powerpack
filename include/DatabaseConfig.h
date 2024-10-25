#ifndef DATABASECONFIG_H
#define DATABASECONFIG_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

class DatabaseConfig {
   public:
    DatabaseConfig(const YAML::Node& config);
    ~DatabaseConfig();
    int get_n_pos_channel() const;
    int get_n_neg_channel() const;
    int get_n_pid_channel() const;
    std::vector<double> get_sensor_parameters() const;
    std::vector<double> get_reference_parameters() const;
    std::vector<double> get_PWM_parameters() const;
    std::vector<double> get_MPC_parameters() const;
    std::vector<double> get_pos_pid_gains() const;
    std::vector<double> get_neg_pid_gains() const;
    std::vector<bool> get_system_parameters() const;
    std::vector<double> get_channel_volume() const;

   private:
    YAML::Node config;
    int n_pos_channel;
    int n_neg_channel;
    int n_pid_channel;
    std::vector<double> sensor_parameters;
    std::vector<double> reference_parameters;
    std::vector<double> PWM_parameters;
    std::vector<double> MPC_parameters;
    std::vector<double> pos_pid_gains;
    std::vector<double> neg_pid_gains;
    std::vector<double> channel_volume;
    std::vector<bool> system_parameters;
};

#endif  // DATABASECONFIG_H