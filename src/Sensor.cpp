#include "Sensor.h"

#include <iostream>
#include <vector>

Sensor::Sensor(ros::NodeHandle& nh,
               std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
      is_initialized(false),
      n_pump_channel(2),
      n_macro_channel(0),
      n_pos_channel(databaseconfig->get_n_pos_channel()),
      n_neg_channel(databaseconfig->get_n_neg_channel()) {
    std::vector<double> parameters = databaseconfig->get_sensor_parameters();
    std::vector<bool> system_parameters = databaseconfig->get_system_parameters();
    frequency = parameters[0];
    pos_offset = parameters[1];
    pos_gain = parameters[2];
    neg_offset = parameters[3];
    neg_gain = parameters[4];
    atm_offset = parameters[5];
    pressure_pos_index = (int)parameters[6];
    pressure_neg_index = (int)parameters[7];
    pressure_macro_index = (int)parameters[8];

    do_initialize = system_parameters[3];

    offset.resize(
        n_pump_channel + n_macro_channel + n_pos_channel + n_neg_channel,
        pos_offset);

    subscriber = nh.subscribe("sen_raw_values", 1,  // 큐 사이즈를 두 번째 인자로 설정
                    &Sensor::subscriber_callback, this,
                    ros::TransportHints().tcpNoDelay());

    data.resize(n_pump_channel + n_macro_channel + n_pos_channel +
                n_neg_channel);

    publisher = nh.advertise<std_msgs::Float32MultiArray>("sen_values", 1);
}

Sensor::~Sensor() {}

void Sensor::initialize(std::vector<double> data_vector, int pressure_macro_index) {
    if (do_initialize) {
        for (int index = 0; index <= data_vector.size(); ++index){ 
            if (index != pressure_macro_index) {
                offset[index] = data_vector[index];
            }
        }
        ROS_INFO("Initialized!! Please turn on the pump");
        sleep(5);
    }
    is_initialized = true;
}

void Sensor::update(const std::vector<double> _data) { data = _data; }

std::vector<double> Sensor::get_data() const { return data; }
