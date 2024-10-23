#include <iostream>
#include <ros/ros.h>


#include "Powerpack.h"


Powerpack::Powerpack(std::vector<double> sensor_parameters, int n_pos_channel, int n_neg_channel, std::vector<double> gains) :
    n_pos_channel(n_pos_channel),
    n_neg_channel(n_neg_channel) {

        
    sensor = std::make_unique<Sensor>(n_pos_channel,n_neg_channel, sensor_parameters);
    pwm = std::make_unique<PWM>(3*(n_pos_channel+n_neg_channel));
    referencegoverner = std::make_unique<ReferenceGoverner>((n_pos_channel+n_neg_channel), 0);
    
    // pwm = new PWM((3*(n_pos_channel+n_neg_channel)));
    // referencegoverner = new ReferenceGoverner((n_pos_channel+n_neg_channel), 0);

    for (int i = 0; i < n_pos_channel; ++i) {
        ControlModule tmp_module(*sensor, *pwm, *referencegoverner, positive,3+i, {3*i, 3*i+1, 3*i+2}, gains);
        modules.push_back(tmp_module);
    }
    for (int i = n_pos_channel; i < n_pos_channel+n_neg_channel; ++i) {
        ControlModule tmp_module(*sensor, *pwm, *referencegoverner, negative, 3+i, {3*i, 3*i+1, 3*i+2}, gains);
        modules.push_back(tmp_module);
    }

    // std::cout << "Powerpack generated" << std::endl;
    print_powerpack_info();

}

Powerpack::~Powerpack() {

}

void Powerpack::get_pos_powerpack_info() {
    for (const auto& tmp_module : modules) {
        if (tmp_module.get_channel_type() == true) {
            tmp_module.get_channel_info();
        }
    }
}

void Powerpack::get_neg_powerpack_info() {
    for (const auto& tmp_module : modules) {
        if (tmp_module.get_channel_type() == false) {
            tmp_module.get_channel_info();
        }
    }
}

void Powerpack::print_powerpack_info() {
    std::cout << "====================" << std::endl;
    std::cout << "channel info" << std::endl;
    std::cout << "number of positive channel : " << n_pos_channel << std::endl;
    std::cout << "number of negative channel : " << n_neg_channel << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "controller type" << std::endl;
    for (const auto& tmp_module : modules) {
        std::cout << tmp_module.get_controller_type() << ", ";
    }
    std::cout << "--------------------" << std::endl;
    std::cout << "module sign" << std::endl;
    for (const auto& tmp_module : modules) {
        tmp_module.get_channel_info();
    }
    std::cout << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "Sensor info" << std::endl;
    std::cout << "number of sensor channel : " << sensor->get_n_channel() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "PWM info" << std::endl;
    std::cout << "number of pwm channel : " << pwm->get_n_channel() << std::endl;
    
    std::cout << "====================" << std::endl;

}

void Powerpack::update_sensor(std::vector<double> _data) {
    sensor->update(_data);
}

void Powerpack::update_reference(std::vector<double> _data) {
    referencegoverner->update(_data);
}

void Powerpack::update_pwm(){
    set_all_control_signal();
    
}

void Powerpack::calculate_control_signal() {
    //sensor -> 0: pos_micro, 1 : neg_micro, 2 : macro
    for (int i = 0; i<(n_pos_channel); ++i) {
        modules[i].calculate_control_signal(sensor->get_data(i+3),sensor->get_data(0),sensor->get_data(2), referencegoverner->get_data(i));
    }
    for( int i = (0 + n_pos_channel); i < (n_pos_channel+n_neg_channel);++i) {
        modules[i].calculate_control_signal(sensor->get_data(i+3),sensor->get_data(1),sensor->get_data(2), referencegoverner->get_data(i));
    } 
}


void Powerpack::set_all_control_signal() {
     std::vector<unsigned int> pwm_signal;
    for (const auto& tmp_module : modules) {
        std::vector<unsigned int> tmp_value = tmp_module.get_control_signal();
        pwm_signal.insert(pwm_signal.end(), tmp_value.begin(), tmp_value.end());
    }
    pwm->update(pwm_signal);
}

std::vector<double> Powerpack::get_all_sensor_data() {
    return sensor->get_all_data();
}

std::vector<double> Powerpack::get_all_reference_data() {
    return referencegoverner->get_all_data();
}


void Powerpack::run() {
    // std::cout << "powerpack run executed" << std::endl;
    // std::cout << "print all sensor" << std::endl;
    // print_data_vector(get_all_sensor_data());
    // std::cout << "print all reference" << std::endl;
    // print_data_vector(get_all_reference_data());
    

    calculate_control_signal();
    update_pwm();

    // std::cout << "cacluate control executed" << std::endl;
    // set_all_control_signal();
    // std::cout << "set pwm executed" << std::endl;
    
    // get_all_powerpack_info();
    // sensor->print_all_data();
    // referencegoverner->print_all_data();
    // pwm->print_all_data();
}

std::vector<unsigned int>  Powerpack::get_control_signal() {
    return pwm->get_control_signal();
}
