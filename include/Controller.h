#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <Eigen/Sparse> 
#include <Eigen/Dense>
#include <memory>
#include <osqp.h> 

#include "Valve.h"
#include "Solver.h"

class Controller {
    public:
        Controller();
        ~Controller();

        void set_now_state(double P_now, double _P_micro, double _P_macro);
        virtual void set_now_target_trajectory(double target) = 0;
       

        std::string get_controller_type();
        std::vector<unsigned int> get_control_signal();

        virtual void calculate_control() =0;
        virtual void print_controller_info() =0;

    protected:
        double P_now;
        double P_micro;
        double P_macro;

        const double P_atm = 101.325;

        std::vector<double> target_trajectory;
        std::vector<unsigned int> control;
        std::string controller_type;
};

class PIDController : public Controller {
    public:
        PIDController(bool is_positive, std::vector<double> gains);
        ~PIDController();

        void calculate_input_reference();

        virtual void calculate_control() override;
        virtual void print_controller_info() override;
        void set_now_target_trajectory(double target) override;

    private:
        bool is_positive;
        double kp_micro;
        double ki_micro;
        double kd_micro;
        double kp_macro;
        double ki_macro;
        double kd_macro;
        double kp_atm;
        double ki_atm;
        double kd_atm;
        double previous_error;
        double cumulated_error;
};

class MPCController : public Controller {
    public:
        MPCController(bool is_positive, std::vector<double> gains);
        ~MPCController();

        virtual void calculate_control() override;
        virtual void print_controller_info() override;

        void set_now_target_trajectory(double target) override;

        double input_mapping(double input, double P_in, double P_out);

        void calculate_A_B_matrix();
        void calculate_input_reference();
        void calculate_P_q_matrix();
        void calculate_constraint_matrix();
        void calculate_upper_triangle_matrix(const Eigen::MatrixXd& input_matrix, Eigen::MatrixXd& upper_triangle_matrix);
        void solve_QP();

    private:
        bool is_positive;
        int NP;
        double Ts;
        double Q_value;
        double R_value;
        double kp_micro;
        double kp_macro;
        double kp_atm;
        int n_x;
        int n_u;

        std::unique_ptr<Solver> solver;

        std::vector<double> A_s;
        std::vector<Eigen::RowVector3d> B_s;
        
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd P;
        Eigen::MatrixXd P_triangle;
        Eigen::MatrixXd q;
        Eigen::MatrixXd constraint_A;
        Eigen::VectorXd UL;
        Eigen::VectorXd LL;
        Eigen::VectorXd input_reference_micro;
        Eigen::VectorXd input_reference_macro;
        Eigen::VectorXd input_reference_atm;

        std::unique_ptr<Valve> valve_micro;
        std::unique_ptr<Valve> valve_macro;
        std::unique_ptr<Valve> valve_atm;

        OSQPInt n;
        OSQPInt m;
};



#endif //CONTROLLER_H