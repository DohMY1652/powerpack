#include <iostream>
#include <ros/ros.h>


#include "Controller.h"



Controller::Controller() {

}

Controller::~Controller() {

}

void Controller::set_now_state(double _P_now, double _P_micro, double _P_macro) {
    P_now = _P_now;
    P_micro = _P_micro;
    P_macro = _P_macro;
}





std::string Controller::get_controller_type() {
    return controller_type;
}

std::vector<unsigned int> Controller::get_control_signal() {
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

void PIDController::set_now_target_trajectory(double target) {
    // ROS_INFO("set now target tarjectory");
    target_trajectory.resize(1,target);
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

MPCController::MPCController(bool is_positive, std::vector<double> gains) :
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

    n = n_u*NP;
    m = n_u*NP;
    
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


void MPCController::set_now_target_trajectory(double target) {
    target_trajectory.resize(NP,target);
}


void MPCController::calculate_input_reference() {
    Eigen::VectorXd target_trajectory_eigen = Eigen::VectorXd::Map(target_trajectory.data(), NP);
    Eigen::VectorXd tmp_vector = Eigen::VectorXd::Constant(NP, P_now);
    Eigen::VectorXd now_error = target_trajectory_eigen - tmp_vector;
    if(is_positive) {
        now_error *= 1;
    }
    else {
        now_error *= -1;
    }

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

double MPCController::input_mapping(double input, double P_in, double P_out) {
    
    double delP = P_in - P_out;
    double u_min = 98.85 - 0.03191 * delP;
    double Q_max = -407.1 + 0.1922 * delP + 4.072 * 100;
    if (input >= Q_max) {
        return 1000;
    }
    else {
        return 10 * (input / Q_max * (100-u_min) + u_min);
    }
}

void MPCController::calculate_A_B_matrix() {
    for (int idx = 0; idx < NP; idx++) {
        double tmp_A;
        Eigen::RowVector3d tmp_B;
        if (is_positive) {
            valve_micro->calculate_valve_dynamic(input_mapping(input_reference_micro[idx],P_micro, P_now),P_micro, P_now);
            valve_macro->calculate_valve_dynamic(input_mapping(input_reference_macro[idx],P_macro, P_now),P_macro, P_now);
            valve_atm->calculate_valve_dynamic(input_mapping(input_reference_atm[idx],P_now, P_atm),P_now, P_atm);
            tmp_A = valve_micro->get_round_pressure_out() +
                    valve_macro->get_round_pressure_out() +
                    -1 * valve_atm->get_round_pressure_in();
            tmp_B << valve_micro->get_round_input(),
                    valve_macro->get_round_input(),
                    -1 * valve_atm->get_round_input();
        }
        else {
            valve_micro->calculate_valve_dynamic(input_mapping(input_reference_micro[idx],P_now, P_micro),P_now, P_micro);
            valve_macro->calculate_valve_dynamic(input_mapping(input_reference_macro[idx],P_now, P_macro),P_now, P_macro);
            valve_atm->calculate_valve_dynamic(input_mapping(input_reference_atm[idx],P_atm, P_now),P_atm, P_now);
            tmp_A = -1 * (valve_micro->get_round_pressure_out() +
                valve_macro->get_round_pressure_out() +
                -1 * valve_atm->get_round_pressure_in());
            tmp_B << -1 * (valve_micro->get_round_input()),
                    -1 * (valve_macro->get_round_input()),
                    valve_atm->get_round_input();
        }
        
        A_s.push_back(tmp_A);
        B_s.push_back(tmp_B);

    }
}

void MPCController::calculate_P_q_matrix() {
    Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_x*NP, n_u*NP);
    for (int i = 0; i < NP; ++i) {
        auto A = A_s[i];
        auto B = B_s[i];
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                int row = i * n_x;
                int column = j * n_u;
                if (B.size() == n_u) {
                    S_bar.block(row, column, n_x,n_u) = B;
                } else {
                    ROS_INFO("Error: Size of B does not match n_u");
                }
            } 
            else if (j < i) {
                int row = i * n_x;
                int column = j * n_u;
                Eigen::RowVector3d tmp_mat = B;
                for (int k = 0; k < i - j; ++k) {
                    tmp_mat = A * tmp_mat;
                }
                if (tmp_mat.size() == n_u) {
                    S_bar.block(row, column, n_x, n_u) = tmp_mat;
                } else {
                    ROS_INFO("Error: Size of tmp_mat does not match n_u");
                }
            }
        }
    }


    Eigen::MatrixXd T_bar = Eigen::MatrixXd::Zero(NP, n_x);
    Eigen::MatrixXd tmp(1, 1);
    tmp(0,0) = 1;
    for(int i = 0; i < NP; ++i) {
        Eigen::MatrixXd A(1,1);
        A(0,0)  = A_s[i];
        tmp = A*tmp;
        int row = i;
        int column = 0;
        T_bar.block(row, column, n_x, n_x) = tmp;
    }
    P = 2*(R+(S_bar.transpose()*Q*S_bar));
    q = P_now*2*T_bar.transpose()*Q*S_bar;   
}


void MPCController::calculate_constraint_matrix() {
    UL.resize(n_u*NP);
    LL.resize(n_u*NP);
    for (int i = 0; i < NP; ++i) {
        UL(i * 3) = 100 - input_reference_micro(i);      
        UL(i * 3 + 1) = 100 - input_reference_macro(i);  
        UL(i * 3 + 2) = 100 - input_reference_atm(i);
        LL(i * 3) = 0 - input_reference_micro(i);      
        LL(i * 3 + 1) = 0 - input_reference_macro(i);  
        LL(i * 3 + 2) = 0 - input_reference_atm(i); 
    }
}

void MPCController::calculate_upper_triangle_matrix(const Eigen::MatrixXd& input_matrix, Eigen::MatrixXd& upper_triangle_matrix) {
    upper_triangle_matrix = input_matrix; 
    int rows = upper_triangle_matrix.rows();
    int cols = upper_triangle_matrix.cols();

    for (int i = 0; i < std::min(rows, cols); ++i) {
        for (int j = i + 1; j < rows; ++j) {
            double factor = upper_triangle_matrix(j, i) / upper_triangle_matrix(i, i);
            upper_triangle_matrix.row(j) -= factor * upper_triangle_matrix.row(i);
        }
    }
}


void MPCController::solve_QP() {
    Eigen::VectorXd q_vector(q.cols());
    q_vector =q.row(0).transpose();
    calculate_upper_triangle_matrix(P, P_triangle);
    try {
        Solver solver(P_triangle, q_vector, constraint_A, LL, UL, n, m);
        OSQPInt exitflag = solver.solve();

        if (exitflag == 0) {
            std::vector<double> tmp = solver.get_result();
            control[0] = (unsigned int)input_mapping((tmp[0]+input_reference_micro(1)), P_micro, P_now);
            control[1] = (unsigned int)input_mapping((tmp[1]+input_reference_macro(1)), P_macro, P_now);
            control[2] = (unsigned int)input_mapping((tmp[2]+input_reference_atm(1)), P_now, P_atm);

            // ROS_INFO("Result: %u", control[0]);
            // ROS_INFO("Result: %u", control[1]);
            // ROS_INFO("Result: %u", control[2]);
        } else {
            ROS_INFO("Optimization failed with exitflag: ");
        }
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
    }
}


void MPCController::calculate_control() {
    calculate_input_reference();
    calculate_A_B_matrix();
    calculate_P_q_matrix();
    calculate_constraint_matrix();
    solve_QP();

}

void MPCController::print_controller_info() {

}

