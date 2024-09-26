#include <iostream>

#include "Sensor.h"

Sensor::Sensor(int n_channel) : n_channel(n_channel) {
    data.resize(n_channel);
    // std::cout << "sensor generated" << std::endl;
}

Sensor::~Sensor() {

}

void Sensor::update() {
    for (int i = 0; i < n_channel; i++) {
        data[i] = double(i);
    }
    std::cout << "sensor updated" << std::endl;
}

double Sensor::get_data(int index) {
    return data[index];
}

int Sensor::get_n_channel() {
    return n_channel;
}


void Sensor::print_all_data() {
    std::cout << "Sensor data : ";
    for (int i = 0; i < n_channel; i++) {
        std::cout << data[i];
        if(i != (n_channel-1)) {
        std::cout << ", ";
        }
    }
    std::cout << std::endl;
}


void ADS1263::update() {
    
}