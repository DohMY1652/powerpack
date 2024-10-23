#include <iostream>

#include "Sensor.h"

Sensor::Sensor(int n_pos_channel, int n_neg_channel, std::vector<double> parameters) : 
    n_pump_channel(2),
    n_macro_channel(1),
    n_pos_channel(n_pos_channel), 
    n_neg_channel(n_neg_channel) {

        pos_offset = parameters[0];
        pos_gain = parameters[1];
        neg_offset = parameters[2];
        neg_gain = parameters[3];
        atm_offset = parameters[4];
        
        data.resize(n_pump_channel + n_macro_channel + n_pos_channel + n_neg_channel);
}

Sensor::~Sensor() {

}

void Sensor::update(std::vector<double> _data) {
    // data = _data;
    data[0] = (_data[0]-pos_offset)*pos_gain + atm_offset;
    data[1] = (_data[1]-neg_offset)*neg_gain + atm_offset;
    data[2] = (_data[2]-pos_offset)*pos_gain + atm_offset;
    data[9] = 0;

    for (int i = 3; i < 3+n_pos_channel; ++i) {
        data[i] = (_data[i]-pos_offset)*pos_gain + atm_offset;
    }
    for (int i = 3+n_pos_channel; i < 3+n_pos_channel+n_neg_channel; ++i) {
        data[i] = (_data[i]-neg_offset)*neg_gain + atm_offset;
    }  

}

double Sensor::get_data(int index) {
    return data[index];
}

std::vector<double> Sensor::get_all_data() {
    return data;
}


int Sensor::get_n_channel() {
    return n_pos_channel+n_neg_channel;
}


void Sensor::print_all_data() {
    std::cout << "Sensor data : ";
    for (int i = 0; i < n_pos_channel+n_neg_channel; i++) {
        std::cout << data[i];
        if(i != (n_pos_channel+n_neg_channel-1)) {
        std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

