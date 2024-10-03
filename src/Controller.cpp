#include <iostream>

#include "Controller.h"

Controller::Controller() {

}

Controller::~Controller() {

}

void Controller::set_now_state(double _now) {
    now = _now;
    
}

void Controller::set_now_target_trajectory(std::vector<double> _target_trajectory) {
    target_trajectory = _target_trajectory;
}


std::string Controller::get_controller_type() {
    return controller_type;
}

std::vector<double> Controller::get_control_signal() {
    return control;
}

PIDController::PIDController(bool is_poisitive, std::vector<double> gains) :
    previous_error(0),
    cumulated_error(0){
        controller_type = "PID";
        control = {0, 0, 0};
            kp_micro = gains[0];
            ki_micro = gains[1];
            kd_micro = gains[2];
            kp_macro = gains[3];
            ki_macro = gains[4];
            kd_macro = gains[5];
            kp_atm = gains[6];
            ki_atm = gains[7];
            kd_atm = gains[8];
       
}

PIDController::~PIDController() {
    
}

void PIDController::print_controller_info() {
    std::cout << "=======================" << std::endl;
    std::cout << "controller type : PID"<< std::endl;
    std::cout << "-------------------" << std::endl;
    std::cout << "micro gains" << std::endl;
    std::cout << "P-gain : " << kp_micro << std::endl;
    std::cout << "I-gain : " << ki_micro << std::endl;
    std::cout << "D-gain : " << kd_micro << std::endl;
    std::cout << "-------------------" << std::endl;
    std::cout << "macro gains" << std::endl;
    std::cout << "P-gain : " << kp_macro << std::endl;
    std::cout << "I-gain : " << ki_macro << std::endl;
    std::cout << "D-gain : " << kd_macro << std::endl;
    std::cout << "-------------------" << std::endl;
    std::cout << "atm gains" << std::endl;
    std::cout << "P-gain : " << kp_atm << std::endl;
    std::cout << "I-gain : " << ki_atm << std::endl;
    std::cout << "D-gain : " << kd_atm << std::endl;
    std::cout << "=======================" << std::endl;

}

void PIDController::calculate_control() {
    double target = target_trajectory[0];
    double error = target-now;
    cumulated_error += error;
    control[0] = kp_micro*error+ki_micro*cumulated_error+kd_micro*(error-previous_error);
    control[1] = kp_macro*error+ki_macro*cumulated_error+kd_macro*(error-previous_error);
    control[2] = 1000-(kp_atm*error+ki_atm*cumulated_error+kd_atm*(error-previous_error));
}