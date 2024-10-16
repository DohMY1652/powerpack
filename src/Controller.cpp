#include <iostream>

#include "Controller.h"
#include <osqp.h> 


Controller::Controller() {

}

Controller::~Controller() {

}

void Controller::set_now_state(double _P_now, double _P_micro, double _P_macro) {
    P_now = _P_now;
    P_micro = _P_micro;
    P_macro = _P_macro;

}

void Controller::set_now_target_trajectory(std::vector<double> _target_trajectory) {
    target_trajectory = _target_trajectory;
}

void Controller::set_now_target_trajectory(double target) {
    target_trajectory.resize(target_trajectory.size(),target);
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
    static int i = 0;
    double target = target_trajectory[0];
    double error = target-P_now;
    cumulated_error += error;
    // control[0] = kp_micro*error+ki_micro*cumulated_error+kd_micro*(error-previous_error);
    // control[1] = kp_macro*error+ki_macro*cumulated_error+kd_macro*(error-previous_error);
    // control[2] = 1000-(kp_atm*error+ki_atm*cumulated_error+kd_atm*(error-previous_error));
    control[0] = i;
    control[1] = i;
    control[2] = i++;
    if (i > 1000) {
        i = 0;
    }
}

MPCController::MPCController(bool is_poisitive, std::vector<double> gains) :
    is_positive(is_positive) {
    controller_type = "MPC";
    control = {0, 0, 0};

    valve_micro = std::make_unique<Valve>();
    valve_macro = std::make_unique<Valve>();
    valve_atm = std::make_unique<Valve>();


    NP = int(gains[0]);
    Ts = gains[1];   
    Q_value = gains[2];
    R_value = gains[3];
    kp_micro = gains[4];
    kp_macro = gains[5];
    kp_atm = gains[6];
    n_x = gains[7];
    n_u = gains[8];

    Q.resize(n_x*NP,n_x*NP);
    Q.setZero();
    Q.diagonal().setConstant(Q_value);
    R.resize(n_u*NP,n_u*NP);
    R.setZero();
    R.diagonal().setConstant(R_value);
    constraint_A.resize(n_u*NP,n_u*NP);
    constraint_A.setZero();
    constraint_A.diagonal().setConstant(1);
}

MPCController::~MPCController() {

}


void MPCController::calculate_input_reference() {
    Eigen::VectorXd target_trajectory_eigen = Eigen::VectorXd::Map(target_trajectory.data(), NP);
    Eigen::VectorXd tmp_vector = Eigen::VectorXd::Constant(NP, P_now);
    Eigen::VectorXd now_error = target_trajectory_eigen - tmp_vector;


    input_reference_micro = kp_micro*now_error;
    input_reference_macro = kp_macro*now_error;
    input_reference_atm = kp_atm*now_error;

    for (int idx = 0; idx < now_error.size(); idx++) {
        if (now_error[idx] < 0) {
            input_reference_micro[idx] = 0;
            input_reference_macro[idx] = 0;
            input_reference_atm[idx] = abs(input_reference_atm[idx]);
        }
        else {
            input_reference_atm[idx] = 0;
        }
    }
}

void MPCController::calculate_A_B_matrix() {
    for (int idx = 0; idx < NP; idx++) {
        double tmp_A;
        Eigen::Vector3d tmp_B;
        valve_micro->calculate_valve_dynamic(input_reference_micro[idx],P_micro, P_now);
        valve_macro->calculate_valve_dynamic(input_reference_macro[idx],P_macro, P_now);
        valve_atm->calculate_valve_dynamic(input_reference_atm[idx],P_now, P_atm);
        tmp_A = valve_micro->get_round_pressure_out() +
                valve_macro->get_round_pressure_out() +
                -1 * valve_atm->get_round_pressure_in();
        tmp_B << valve_micro->get_round_input(),
                 valve_macro->get_round_input(),
                 -1 * valve_atm->get_round_input();
        A_s.push_back(tmp_A);
        B_s.push_back(tmp_B);
    }
}

void MPCController::calculate_P_q_matrix() {
    Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_x*NP, n_u*NP);
    for (int i = 0; i < A_s.size(); ++i) {
        double A  = A_s[i];
        Eigen::Vector3d B = B_s[i];
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                int row = i;
                int column = i*n_u;
                S_bar.block(row, column, n_x, B.size()) = B;
            }
            else if (j < i) {
                int row = i;
                int column = j*n_u;
                Eigen::Vector3d tmp_mat = B;
                for (int k = 0; k <= i-j; ++k) {
                    tmp_mat  = A * tmp_mat;
                }
                 S_bar.block(row, column, n_x, B.size()) = tmp_mat;
            }
        }
    }
    Eigen::MatrixXd T_bar = Eigen::MatrixXd::Zero(NP, n_x);
    Eigen::MatrixXd tmp(1, 1);
    for(int i = 0; i < A_s.size(); ++i) {
        double A  = A_s[i];
        tmp = A*tmp;
        int row = i;
        int column = 1;
        T_bar.block(row, column, n_x, n_x) = tmp;
    }
    
    P = 2*(R+(S_bar.transpose()*Q*S_bar));
    q = P_now*2*T_bar.transpose()*Q*S_bar;   

}


void MPCController::calculate_constraint_matrix() {
    UL.resize(n_u*NP,1);
    LL.resize(n_u*NP,1);
    for (int i = 0; i < NP; ++i) {
        UL(i * 3) = 100 - input_reference_micro(i);      
        UL(i * 3 + 1) = 100 - input_reference_macro(i);  
        UL(i * 3 + 2) = 100 - input_reference_atm(i);
        LL(i * 3) = 0 - input_reference_micro(i);      
        LL(i * 3 + 1) = 0 - input_reference_macro(i);  
        LL(i * 3 + 2) = 0 - input_reference_atm(i); 
    }
}

void MPCController::solve_QP() {
    
}

void MPCController::calculate_control() {
    calculate_input_reference();
    calculate_A_B_matrix();
    calculate_P_q_matrix();
    calculate_constraint_matrix();
    //set Constraints
    //solve QP
    //set control data


     static int i = 0;
     control[0] = i;
    control[1] = i;
    control[2] = i++;
    if (i > 1000) {
        i = 0;
    }

}

void MPCController::print_controller_info() {

}

