#include "Solver.h"

#include <iostream>

Solver::Solver(bool is_positive, std::shared_ptr<Sensor> &sensor,
               std::shared_ptr<ReferenceGoverner> &referencegoverner,
               std::shared_ptr<DatabaseConfig> &databaseconfig,
               std::shared_ptr<QP>& qp)
    : sensor(sensor),
      referencegoverner(referencegoverner),
      databaseconfig(databaseconfig),
      is_positive(is_positive),
      qp(qp) {
    std::vector<double> parameters = databaseconfig->get_MPC_parameters();
    NP = (int)parameters[0];
    n_x = (int)parameters[1];
    n_u = (int)parameters[2];
    Ts = parameters[3];
    Q_value = parameters[4];
    R_value = parameters[5];
    pos_ku_micro = parameters[6];
    pos_ku_macro = parameters[7];
    pos_ku_atm = parameters[8];
    neg_ku_micro = parameters[9];
    neg_ku_macro = parameters[10];
    neg_ku_atm = parameters[11];

    valve_micro = std::make_unique<Dynamics>(databaseconfig);
    valve_macro = std::make_unique<Dynamics>(databaseconfig);
    valve_atm = std::make_unique<Dynamics>(databaseconfig);

    n = n_u * NP;
    m = n_u * NP;
    
    result.resize(n_u,0);

    Q.resize(n_x * NP, n_x * NP);
    Q.setZero();
    Q.diagonal().setConstant(Q_value);
    R.resize(n_u * NP, n_u * NP);
    R.setZero();
    R.diagonal().setConstant(R_value);
    constraint_A.resize(n_u * NP, n_u * NP);
    constraint_A.setZero();
    constraint_A.diagonal().setConstant(1);

}

Solver::~Solver() {}

bool Solver::get_type() const { return is_positive; }

int Solver::get_sensor_channel() const { return sensor_channel; }

std::vector<double> Solver::get_result() const { return result; }

std::vector<double> Solver::get_pressures() const {
    return {P_now, P_micro, P_macro};
}

void Solver::set_sensor_channel(int _sensor_channel) {
    sensor_channel = _sensor_channel;
}

void Solver::set_reference_channel(int _reference_channel) {
    reference_channel = _reference_channel;
}

void Solver::update_pressure() {
    std::vector<double> sensor_values = sensor->get_data();
    if (is_positive) {
        P_now = sensor_values[sensor_channel];
        P_micro = sensor_values[0];
        P_macro = sensor_values[2];
        std::cout << P_now << " "<< P_micro << " "<< P_macro << std::endl;
    } else {
        P_now = sensor_values[sensor_channel];
        P_micro = sensor_values[1];
        P_macro = sensor_values[2];
        std::cout << P_now << " "<< P_micro << " "<< P_macro << std::endl;
    }
}

void Solver::update_reference() {
    P_ref.resize(NP, referencegoverner->get_data()[reference_channel]);
}

void Solver::calculate_input_reference() {
    Eigen::VectorXd target_trajectory = Eigen::VectorXd::Map(P_ref.data(), NP);
    Eigen::VectorXd const_P_now = Eigen::VectorXd::Constant(NP, P_now);
    Eigen::VectorXd now_error = target_trajectory - const_P_now;
    if (is_positive) {
        now_error *= 1;
        input_reference_micro = pos_ku_micro * now_error;
        input_reference_macro = pos_ku_macro * now_error;
        input_reference_atm = pos_ku_atm * now_error;
    } else {
        now_error *= -1;
        input_reference_micro = neg_ku_micro * now_error;
        input_reference_macro = neg_ku_macro * now_error;
        input_reference_atm = neg_ku_atm * now_error;
    }

    for (int idx = 0; idx < now_error.size(); idx++) {
        if (now_error[idx] < 0) {
            input_reference_micro[idx] = 0;
            input_reference_macro[idx] = 0;
            input_reference_atm[idx] = abs(input_reference_atm[idx]);
        } else {
            input_reference_atm[idx] = 0;
        }
        assert(input_reference_micro[idx] >= 0);
        assert(input_reference_macro[idx] >= 0);
        assert(input_reference_atm[idx] >= 0);
    }

    for(int i = 0; i < NP; ++i) {
        input_reference_micro[i] = input_mapping(input_reference_micro[i],P_micro, P_now);
        input_reference_macro[i] = input_mapping(input_reference_macro[i],P_macro, P_now);
        input_reference_atm[i] = input_mapping (input_reference_atm[i],P_now, P_atm);
    }

    U_ref = {input_reference_micro[0], input_reference_macro[0], input_reference_atm[0]};

    // ROS_INFO("===================================");
    // std::cout << input_reference_micro <<std::endl;
    // ROS_INFO("----------------------------------");
    // std::cout << input_reference_macro <<std::endl;
    // ROS_INFO("----------------------------------");
    // std::cout << input_reference_atm <<std::endl;

}

double Solver::input_mapping(double input, double P_in, double P_out) {
    double delP = abs(P_in - P_out);
    double u_min = 98.85 - 0.03191 * delP;
    double Q_max = -407.1 + 0.1922 * delP + 4.072 * 100;
    if (input >= Q_max) {
        return 100;
    } else if (input < 10) { // 10 is tuning parameters, minimum values of valve open command
        return u_min-20; // 20 is tuning parameters. lower u_min
    } else {
        return (input / Q_max * (100 - u_min) + u_min);
    }
}

void Solver::calculate_A_B_matrix() {
    A_s.resize(0);
    B_s.resize(0);
    A_s.reserve(NP);
    B_s.reserve(NP);
    for (int idx = 0; idx < NP; idx++) {
        double tmp_A;
        Eigen::RowVector3d tmp_B;
        double volume =
            (double)(databaseconfig->get_channel_volume()[reference_channel] *
                     1.0_m3);
        if (is_positive) {
            valve_micro->calculate_valve_dynamic(
                input_reference_micro[idx], P_micro, P_now, volume);
            valve_macro->calculate_valve_dynamic(
                input_reference_macro[idx], P_macro, P_now, volume);
            valve_atm->calculate_valve_dynamic(
                input_reference_atm[idx], P_now, P_atm, volume);
            tmp_A = valve_micro->get_round_pressure_out() +
                    valve_macro->get_round_pressure_out() +
                    -1 * valve_atm->get_round_pressure_in();
            tmp_B << valve_micro->get_round_input(),
                valve_macro->get_round_input(),
                -1 * valve_atm->get_round_input();
        } else {
            valve_micro->calculate_valve_dynamic(
                input_reference_micro[idx], P_now, P_micro, volume);
            // valve_macro->calculate_valve_dynamic(
            //     input_reference_macro[idx], P_now, P_macro, volume);
            valve_macro->calculate_valve_dynamic(
                input_reference_macro[idx], P_now, 11.325, volume);
            valve_atm->calculate_valve_dynamic(
                input_reference_atm[idx], P_atm, P_now, volume);
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

void Solver::calculate_P_q_matrix() {
    Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_x * NP, n_u * NP);
    for (int i = 0; i < NP; ++i) {
        auto A = A_s[i];
        auto B = B_s[i];
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                int row = i * n_x;
                int column = j * n_u;
                if (B.size() == n_u) {
                    S_bar.block(row, column, n_x, n_u) = B;
                } else {
                    ROS_INFO("Error: Size of B does not match n_u");
                }
            } else if (j < i) {
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
    tmp(0, 0) = 1;
    for (int i = 0; i < NP; ++i) {
        Eigen::MatrixXd A(1, 1);
        A(0, 0) = A_s[i];
        tmp = A * tmp;
        int row = i;
        int column = 0;
        T_bar.block(row, column, n_x, n_x) = tmp;
    }
    P = 2 * (R + (S_bar.transpose() * Q * S_bar));
    P = (P+P.transpose())/2;
    q = P_now * 2 * T_bar.transpose() * Q * S_bar;
}

void Solver::calculate_constraint_matrix() {
    UL.resize(n_u * NP);
    LL.resize(n_u * NP);
    for (int i = 0; i < NP; ++i) {
        UL(i * 3) = 100 - input_reference_micro(i);
        UL(i * 3 + 1) = 100 - input_reference_macro(i);
        UL(i * 3 + 2) = 100 - input_reference_atm(i);
        LL(i * 3) = 0 - input_reference_micro(i);
        LL(i * 3 + 1) = 0 - input_reference_macro(i);
        LL(i * 3 + 2) = 0 - input_reference_atm(i);
    }
}

void Solver::calculate_upper_triangle_matrix(
    const Eigen::MatrixXd &input_matrix,
    Eigen::MatrixXd &upper_triangle_matrix) {
    upper_triangle_matrix = input_matrix;

    ////////////////////////////////////////////////////////////////
    int rows = upper_triangle_matrix.rows();
    int cols = upper_triangle_matrix.cols();

    for (int i = 0; i < std::min(rows, cols); ++i) {
        if (upper_triangle_matrix(i, i) != 0) {
            for (int j = i + 1; j < rows; ++j) {
                double factor = upper_triangle_matrix(j, i) / upper_triangle_matrix(i, i);
                upper_triangle_matrix.row(j) -= factor * upper_triangle_matrix.row(i);
            }
        } else {
            // std::cerr << "Warning: Zero pivot encountered at row " << i << ". The matrix may be singular." << std::endl;
        }
    }
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < i; ++j) {
            upper_triangle_matrix(i, j) = 0; 
        }
    }
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    // 원본 행렬을 상삼각행렬로 변환할 upper_triangle_matrix 초기화
    // upper_triangle_matrix = input_matrix;

    // int rows = upper_triangle_matrix.rows();
    // int cols = upper_triangle_matrix.cols();

    // // 상삼각행렬로 변환
    // for (int i = 0; i < std::min(rows, cols); ++i) {
    //     for (int j = i + 1; j < rows; ++j) {
    //         // 주대각선 요소로 나누어 아래 행을 업데이트
    //         if (upper_triangle_matrix(i, i) != 0) {
    //             double factor = upper_triangle_matrix(j, i) / upper_triangle_matrix(i, i);
    //             upper_triangle_matrix.row(j) -= factor * upper_triangle_matrix.row(i);
    //         }
    //     }
    // }

    // // 대각선 아래의 요소를 0으로 설정
    // for (int i = 1; i < rows; ++i) {
    //     for (int j = 0; j < i; ++j) {
    //         upper_triangle_matrix(i, j) = 0;
    //     }
    // }
    ///////////////////////////////////////////////////////////
}

void Solver::set_result() {
    std::vector<double> raw_result = qp->get_raw_result();
    assert(U_ref.size() == raw_result.size());
    assert(result.size() == raw_result.size());
    
    for( int i = 0; i < n_u; ++i) {
        if ((U_ref[i] + raw_result[i]) <= 100 && (U_ref[i] + raw_result[i]) >= 0) {
            result[i] = (U_ref[i] + raw_result[i]);
        }
        
    }

}

void Solver::run_QP() {
    qp->set_data(P_triangle, q, constraint_A, LL, UL);
    qp->solve();
    set_result();
}

void Solver::solve_QP() {
    calculate_input_reference();
    calculate_A_B_matrix();
    calculate_P_q_matrix();
    calculate_upper_triangle_matrix(P, P_triangle);
    calculate_constraint_matrix();
    run_QP();
}

void Solver::run() {
    update_pressure();
    update_reference();
    solve_QP();
}