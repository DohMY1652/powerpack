#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <memory>
#include <vector>

#include "DatabaseConfig.h"
#include "Dynamics.h"
#include "QP.h"
#include "ReferenceGoverner.h"
#include "Sensor.h"
#include "Units.h"

class Solver {
   public:
    Solver(bool is_positive, std::shared_ptr<Sensor>& sensor,
           std::shared_ptr<ReferenceGoverner>& referencegoverner,
           std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Solver();

    bool get_type() const;
    int get_sensor_channel() const;
    std::vector<double> get_pressures() const;

    void set_sensor_channel(int _sensor_channel);
    void set_reference_channel(int _reference_channel);

    void update_pressure();
    void update_reference();
    void calculate_input_reference();

    double input_mapping(double input, double P_in, double P_out);

    void calculate_A_B_matrix();

    void solve_QP();
    void run();

   private:
    std::shared_ptr<Sensor>& sensor;
    std::shared_ptr<ReferenceGoverner>& referencegoverner;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    int sensor_channel;
    int reference_channel;
    int NP;
    int n_x;
    int n_u;
    double Ts;
    double Q;
    double R;
    double pos_ku_micro;
    double pos_ku_macro;
    double pos_ku_atm;
    double neg_ku_micro;
    double neg_ku_macro;
    double neg_ku_atm;

    bool is_positive;

    const double P_atm = 101.325;  // kPa

    double P_now = 0;
    double P_micro = 0;
    double P_macro = 0;
    std::vector<double> P_ref;
    std::vector<double> U_ref;

    std::vector<double> A_s;
    std::vector<Eigen::RowVector3d> B_s;

    Eigen::VectorXd input_reference_micro;
    Eigen::VectorXd input_reference_macro;
    Eigen::VectorXd input_reference_atm;

    std::unique_ptr<Dynamics> valve_micro;
    std::unique_ptr<Dynamics> valve_macro;
    std::unique_ptr<Dynamics> valve_atm;
    // std::unique_ptr<QP> qp;
};

#endif  // SOLVER_H