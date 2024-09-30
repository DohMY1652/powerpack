#include <iostream>

#include "ReferenceGoverner.h"

ReferenceGoverner::ReferenceGoverner(int n_channel, int id) :
    n_channel(n_channel),
    id(id) {
        data.resize(n_channel);
        // std::cout << "Reference governer generated" << std::endl;
}

ReferenceGoverner::~ReferenceGoverner() {

}

void ReferenceGoverner::update() {
    for (int i = 0; i < n_channel; i++) {
        data[i] = std::vector<double> {double(i*2), double(i*4), double(i*6)};
    }
    std::cout << "reference updated" << std::endl;
}

void ReferenceGoverner::print_all_data() {
    std::cout << "Reference data : ";
    for (int i = 0; i < n_channel; i++) {
        for(const double& value : data[i]) {
        std::cout << value << " ";
        }
        if(i != (n_channel-1)) {
        std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

std::vector<double> ReferenceGoverner::get_data(int idx) {
    return data[idx];
}