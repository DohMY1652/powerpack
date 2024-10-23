#ifndef CONTROLMODULE_H
#define CONTROLMODULE_H

#include <vector>

#include "Sensor.h"
#include "PWM.h"
#include "ReferenceGoverner.h"
#include "Controller.h"

#define positive true
#define negative false

class ControlModule {
    public:

        ControlModule(Sensor& sensor, PWM& pwm, ReferenceGoverner& referencegoverner, bool is_positive, int sensor_idx, std::vector<int> pwm_idx, std::vector<double> gains);
        ~ControlModule();

        void get_channel_info() const;
        bool get_channel_type() const;
        void calculate_control_signal(double now, double P_micro, double P_macro, double target);
        std::vector<unsigned int> get_control_signal() const;
        std::string get_controller_type() const;

    private:
        Sensor& sensor;
        PWM& pwm;
        ReferenceGoverner& referencegoverner;
        // std::unique_ptr<Controller> controller;
        MPCController* controller;
        bool is_positive;
        int sensor_idx;
        std::vector<int> pwm_idx;
        std::vector<unsigned int> control;

};

#endif //CONTROLNODULE_H