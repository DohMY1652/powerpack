#include <iostream>

#include "ReferenceGoverner.h"

ReferenceGoverner::ReferenceGoverner(int n_channel, int id) :
    n_channel(n_channel),
    id(id) {
        data.resize(n_channel);
}

ReferenceGoverner::~ReferenceGoverner() {

}

void ReferenceGoverner::update(std::vector<double> _data) {
   data = _data;
    
}


double ReferenceGoverner::get_data(int index) {
    return data[index];
}

std::vector<double> ReferenceGoverner::get_all_data() {
    return data;

}