#include <iostream>
#include <yaml-cpp/yaml.h>
#include <vector>

class DatabaseConfig {
    public:

        DatabaseConfig(const YAML::Node& config);
        ~DatabaseConfig();
        int get_n_pos_channel();
        int get_n_neg_Channel();
        std::vector<double> get_pos_pid_gains();
        std::vector<double> get_neg_pid_gains();
        std::vector<double> get_MPC_parameters();
        std::vector<double> get_sensor_parameters();


    private:
        YAML::Node config;
        int n_pos_channel;
        int n_neg_channel;
        std::vector<double> pos_pid_gains;
        std::vector<double> neg_pid_gains;
        std::vector<double> MPC_parameters;
        std::vector<double> sensor_parameters;


};