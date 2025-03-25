#include "ReferenceGoverner.h"

#include <iostream>

ReferenceGoverner::ReferenceGoverner(
    ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
      n_channel(databaseconfig->get_n_pos_channel() +
                databaseconfig->get_n_neg_channel()),
      frequency(databaseconfig->get_reference_parameters()[0]) {
    data.resize(n_channel, 101.325);

    subscriber = nh.subscribe("mpc_ref_values", 1,  // 큐 사이즈를 두 번째 인자로 설정
                        &ReferenceGoverner::subscriber_callback, this,
                        ros::TransportHints().tcpNoDelay());



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