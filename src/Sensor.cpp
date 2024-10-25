#include "Sensor.h"

#include <iostream>
#include <vector>

Sensor::Sensor(ros::NodeHandle& nh,
               std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
      is_initialized(false),
      n_pump_channel(2),
      n_macro_channel(1),
      n_pos_channel(databaseconfig->get_n_pos_channel()),
      n_neg_channel(databaseconfig->get_n_neg_channel()) {
    std::vector<double> parameters = databaseconfig->get_sensor_parameters();
    frequency = parameters[0];
    pos_offset = parameters[1];
    pos_gain = parameters[2];
    neg_offset = parameters[3];
    neg_gain = parameters[4];
    atm_offset = parameters[5];
    pressure_pos_index = (int)parameters[6];
    pressure_neg_index = (int)parameters[7];
    pressure_macro_index = (int)parameters[8];
    offset.resize(
        n_pump_channel + n_macro_channel + n_pos_channel + n_neg_channel,
        pos_offset);

    subscriber = nh.subscribe("sen_values", frequency,
                              &Sensor::subscriber_callback, this);
    data.resize(n_pump_channel + n_macro_channel + n_pos_channel +
                n_neg_channel);
}

Sensor::~Sensor() {}

void Sensor::initialize(std::vector<double> data_vector) {
    if (!is_initialized) {
        offset = data_vector;
        // double sum = 0;
        // for (int index = 0; index < offset.size(); ++index) {
        //     if (index != 2) {
        //         sum += offset[index];
        //     }
        // }
        // offset[2] = sum / (offset.size() - 1);
    }
    is_initialized = true;
}

void Sensor::update(const std::vector<double> _data) { data = _data; }

std::vector<double> Sensor::get_data() const { return data; }

// void Sensor::update(std::vector<double> _data) {
//     // data = _data;
//     data[0] = (_data[0]-pos_offset)*pos_gain + atm_offset;
//     data[1] = (_data[1]-neg_offset)*neg_gain + atm_offset;
//     data[2] = (_data[2]-pos_offset)*pos_gain + atm_offset;
//     data[9] = 0;

//     for (int i = 3; i < 3+n_pos_channel; ++i) {
//         data[i] = (_data[i]-pos_offset)*pos_gain + atm_offset;
//     }
//     for (int i = 3+n_pos_channel; i < 3+n_pos_channel+n_neg_channel; ++i) {
//         data[i] = (_data[i]-neg_offset)*neg_gain + atm_offset;
//     }

// }

// double Sensor::get_data(int index) {
//     return data[index];
// }

// std::vector<double> Sensor::get_all_data() {
//     return data;
// }

// int Sensor::get_n_channel() {
//     return n_pos_channel+n_neg_channel;
// }

// void Sensor::print_all_data() {
//     std::cout << "Sensor data : ";
//     for (int i = 0; i < n_pos_channel+n_neg_channel; i++) {
//         std::cout << data[i];
//         if(i != (n_pos_channel+n_neg_channel-1)) {
//         std::cout << ", ";
//         }
//     }
//     std::cout << std::endl;
// }
