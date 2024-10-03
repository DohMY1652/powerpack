#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Powerpack.h"


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>" << std::endl;
        return 1;
    }
    std::string filename = argv[1];
    YAML::Node config = YAML::LoadFile(filename);
    DatabaseConfig data(config);
    
    Powerpack powerpack(data.get_n_pos_channel(), data.get_n_neg_Channel(), data.get_pos_pid_gains(), data.get_neg_pid_gains());
    
    powerpack.run();
}


