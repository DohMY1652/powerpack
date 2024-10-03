#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
// #include <Eigen/Dense>

class Controller {
    public:
        Controller();
        ~Controller();

        void set_now_state(double now);
        void set_now_target_trajectory(std::vector<double> target_trajectory);

        std::string get_controller_type();
        std::vector<double> get_control_signal();

        virtual void calculate_control() =0;
        virtual void print_controller_info() =0;

    protected:
        double now;
        std::vector<double> target_trajectory;
        std::vector<double> control;
        std::string controller_type;
};

class PIDController : public Controller {
    public:
        PIDController(bool is_positive, std::vector<double> gains);
        ~PIDController();

        virtual void calculate_control() override;
        virtual void print_controller_info() override;

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



#endif //CONTROLLER_H