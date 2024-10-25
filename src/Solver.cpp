#include "Solver.h"

#include <iostream>

Solver::Solver(bool is_positive, std::shared_ptr<Sensor> &sensor,
               std::shared_ptr<ReferenceGoverner> &referencegoverner,
               std::shared_ptr<DatabaseConfig> &databaseconfig)
    : sensor(sensor),
      referencegoverner(referencegoverner),
      databaseconfig(databaseconfig),
      is_positive(is_positive) {
    std::vector<double> parameters = databaseconfig->get_MPC_parameters();
    NP = (int)parameters[0];
    n_x = (int)parameters[1];
    n_u = (int)parameters[2];
    Ts = parameters[3];
    Q = parameters[4];
    R = parameters[5];
    pos_ku_micro = parameters[6];
    pos_ku_macro = parameters[7];
    pos_ku_atm = parameters[8];
    neg_ku_micro = parameters[9];
    neg_ku_macro = parameters[10];
    neg_ku_atm = parameters[11];

    valve_micro = std::make_unique<Dynamics>(databaseconfig);
    valve_macro = std::make_unique<Dynamics>(databaseconfig);
    valve_atm = std::make_unique<Dynamics>(databaseconfig);
    // qp = std::make_unique<QP>(databaseconfig);
}

Solver::~Solver() {}

bool Solver::get_type() const { return is_positive; }

int Solver::get_sensor_channel() const { return sensor_channel; }

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
    } else {
        P_now = sensor_values[sensor_channel];
        P_micro = sensor_values[1];
        P_macro = sensor_values[2];
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
}

double Solver::input_mapping(double input, double P_in, double P_out) {
    double delP = P_in - P_out;
    double u_min = 98.85 - 0.03191 * delP;
    double Q_max = -407.1 + 0.1922 * delP + 4.072 * 100;
    if (input >= Q_max) {
        return 100;
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
                input_mapping(input_reference_micro[idx], P_micro, P_now),
                P_micro, P_now, volume);
            valve_macro->calculate_valve_dynamic(
                input_mapping(input_reference_macro[idx], P_macro, P_now),
                P_macro, P_now, volume);
            valve_atm->calculate_valve_dynamic(
                input_mapping(input_reference_atm[idx], P_now, P_atm), P_now,
                P_atm, volume);
            tmp_A = valve_micro->get_round_pressure_out() +
                    valve_macro->get_round_pressure_out() +
                    -1 * valve_atm->get_round_pressure_in();
            tmp_B << valve_micro->get_round_input(),
                valve_macro->get_round_input(),
                -1 * valve_atm->get_round_input();
        } else {
            valve_micro->calculate_valve_dynamic(
                input_mapping(input_reference_micro[idx], P_now, P_micro),
                P_now, P_micro, volume);
            valve_macro->calculate_valve_dynamic(
                input_mapping(input_reference_macro[idx], P_now, P_macro),
                P_now, P_macro, volume);
            valve_atm->calculate_valve_dynamic(
                input_mapping(input_reference_atm[idx], P_atm, P_now), P_atm,
                P_now, volume);
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

void Solver::solve_QP() {
    calculate_input_reference();
    calculate_A_B_matrix();
    // calculate_P_q_matrix();
    // calculate_constraint_matrix();
    // run_QP();
}

void Solver::run() {
    update_pressure();
    update_reference();
    solve_QP();
}