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

void ReferenceGoverner::update(std::vector<std::vector<double>> _data) {
    data = _data;
}
void ReferenceGoverner::update(std::vector<double> _data) {
    std::vector<double> tmp;
    for (int idx = 0 ; idx < data.size(); idx++) {
        if (idx <= _data.size()) {
            tmp = std::vector<double>(data[0].size(), _data[idx]);
        }
        else {
            tmp =  std::vector<double>(data[0].size(), 101.325);
        }
        data[idx] = tmp;
    }
    
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