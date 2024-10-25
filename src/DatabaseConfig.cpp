#include "DatabaseConfig.h"

DatabaseConfig::DatabaseConfig(const YAML::Node& config) : config(config) {
    n_pos_channel = config["positive_channel"]["n_channel"].as<int>();
    n_neg_channel = config["negative_channel"]["n_channel"].as<int>();
    n_pid_channel = config["PID_parameters"]["n_channel"].as<int>();
    for (const auto& data : config["Sensor_parameters"]) {
        sensor_parameters.push_back(data.second.as<double>());
    }
    for (const auto& data : config["Reference_parameters"]) {
        reference_parameters.push_back(data.second.as<double>());
    }
    for (const auto& data : config["PWM_parameters"]) {
        PWM_parameters.push_back(data.second.as<int>());
    }
    for (const auto& data : config["MPC_parameters"]) {
        MPC_parameters.push_back(data.second.as<double>());
    }
    for (const auto& data : config["PID_parameters"]["pos_PID_gains"]) {
        pos_pid_gains.push_back(data.second.as<double>());
    }
    for (const auto& data : config["PID_parameters"]["neg_PID_gains"]) {
        neg_pid_gains.push_back(data.second.as<double>());
    }
    for (const auto& data : config["channel_volume"]) {
        channel_volume.push_back(data.second.as<double>());
    }
    for (const auto& data : config["system_parameters"]) {
        system_parameters.push_back(data.second.as<bool>());
    }
}

DatabaseConfig::~DatabaseConfig() {}

int DatabaseConfig::get_n_pos_channel() const { return n_pos_channel; }

int DatabaseConfig::get_n_neg_channel() const { return n_neg_channel; }

int DatabaseConfig::get_n_pid_channel() const { return n_pid_channel; }

std::vector<double> DatabaseConfig::get_sensor_parameters() const {
    return sensor_parameters;
}
std::vector<double> DatabaseConfig::get_reference_parameters() const {
    return reference_parameters;
}

std::vector<double> DatabaseConfig::get_PWM_parameters() const {
    return PWM_parameters;
}

std::vector<double> DatabaseConfig::get_MPC_parameters() const {
    return MPC_parameters;
}

std::vector<double> DatabaseConfig::get_pos_pid_gains() const {
    return pos_pid_gains;
}

std::vector<double> DatabaseConfig::get_neg_pid_gains() const {
    return neg_pid_gains;
}

std::vector<double> DatabaseConfig::get_channel_volume() const {
    return channel_volume;
}

std::vector<bool> DatabaseConfig::get_system_parameters() const {
    return system_parameters;
}
