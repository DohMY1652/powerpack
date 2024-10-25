#include "ReferenceGoverner.h"

#include <iostream>

ReferenceGoverner::ReferenceGoverner(
    ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
      n_channel(databaseconfig->get_n_pos_channel() +
                databaseconfig->get_n_neg_channel()),
      frequency(databaseconfig->get_reference_parameters()[0]) {
    data.resize(n_channel, 101.325);

    data[0] = 201.325;
    data[1] = 201.325;
    data[2] = 201.325;
    data[3] = 51.325;
    data[4] = 51.325;
    data[5] = 51.325;

    subscriber = nh.subscribe("ref_values", frequency,
                              &ReferenceGoverner::subscriber_callback, this);
    data.resize(n_channel);
}

ReferenceGoverner::~ReferenceGoverner() {}

void ReferenceGoverner::update(const std::vector<double> _data) {
    data = _data;
}

std::vector<double> ReferenceGoverner::get_data() const { return data; }
// void ReferenceGoverner::update(std::vector<double> _data) {
//    data = _data;

// }

// double ReferenceGoverner::get_data(int index) {
//     return data[index];
// }

// std::vector<double> ReferenceGoverner::get_all_data() {
//     return data;

// }