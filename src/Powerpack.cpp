#include "Powerpack.h"

#include <ros/ros.h>

#include <iostream>

#define positive true
#define negative false

Powerpack::Powerpack(ros::NodeHandle& nh,
                     std::shared_ptr<DatabaseConfig>& databaseconfig)
    : nh(nh), databaseconfig(databaseconfig) {
    sensor = std::make_shared<Sensor>(nh, databaseconfig);
    referencegoverner = std::make_shared<ReferenceGoverner>(nh, databaseconfig);
    pwm = std::make_shared<PWM>(nh, databaseconfig);
    qp = std::make_shared<QP>(databaseconfig);
    solvers.reserve(databaseconfig->get_n_pos_channel() +
                    databaseconfig->get_n_neg_channel());
    for (int i = 0; i < databaseconfig->get_n_pos_channel(); ++i) {
        auto tmp_solver = std::make_unique<Solver>(
            positive, sensor, referencegoverner, databaseconfig, qp);
        solvers.push_back(std::move(tmp_solver));
    }
    for (int i = 0; i < databaseconfig->get_n_neg_channel(); ++i) {
        auto tmp_solver = std::make_unique<Solver>(
            negative, sensor, referencegoverner, databaseconfig, qp);
        solvers.push_back(std::move(tmp_solver));
    }
    int channel = 0;
    for (const auto& solver : solvers) {
        solver->set_sensor_channel(channel +
                                   databaseconfig->get_n_pid_channel());
        solver->set_reference_channel(channel);
        channel++;
    }
}

Powerpack::~Powerpack() {}

std::vector<double> Powerpack::get_sensor_data() const {
    return sensor->get_data();
}

std::vector<double> Powerpack::get_reference_data() const {
    return referencegoverner->get_data();
}

void Powerpack::set_pwm() { pwm->update_pwm(pwm_values); }

void Powerpack::run() {
    pwm_values.reserve(solvers.size()*3);
    pwm_values.resize(0);
    for (const auto& solver : solvers) {
        solver->run();
        std::vector<double> tmp = solver->get_result();
        pwm_values.push_back(tmp[0]);
        pwm_values.push_back(tmp[1]);
        pwm_values.push_back(tmp[2]);
    }
     std::for_each(pwm_values.begin(), pwm_values.end(), [](double value) {
        std::cout << value << " "; 
    });

    std::cout << std::endl;
    set_pwm();
}
