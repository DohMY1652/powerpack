#include "DatabaseConfig.h"

DatabaseConfig::DatabaseConfig(const YAML::Node& config) : 
    config(config) {
        n_pos_channel = config["positive_channel"]["n_channel"].as<int>();
        n_neg_channel = config["negative_channel"]["n_channel"].as<int>();
        // for (const auto& gain : config["pos_pid_gains"]) {
        //     pos_pid_gains.push_back(gain.begin()->second.as<double>());
        // }
        for (const auto& gain : config["pos_pid_gains"]) {
                pos_pid_gains.push_back(gain.second.as<double>());
            }
       for (const auto& gain : config["neg_pid_gains"]) {
                neg_pid_gains.push_back(gain.second.as<double>());
            }

        for (const auto& gain : config["MPC_parameters"]) {
                MPC_parameters.push_back(gain.second.as<double>());
        }
        for (const auto& gain : config["Sensor_parameters"]) {
                sensor_parameters.push_back(gain.second.as<double>());
        }
        // pos_pid_gains = config["pos_pid_gains"].as<std::vector<double>>();
        // neg_pid_gains = config["neg_pid_gains"].as<std::vector<double>>();
    }

DatabaseConfig::~DatabaseConfig() {

}

int DatabaseConfig::get_n_pos_channel() {
    return n_pos_channel;
}

int DatabaseConfig::get_n_neg_Channel() {
    return n_neg_channel;
}

std::vector<double> DatabaseConfig::get_pos_pid_gains() {
    return pos_pid_gains;
}

std::vector<double> DatabaseConfig::get_neg_pid_gains() {
    return neg_pid_gains;
}

std::vector<double> DatabaseConfig::get_MPC_parameters(){
    return MPC_parameters;
}

std::vector<double> DatabaseConfig::get_sensor_parameters(){
    return sensor_parameters;
}