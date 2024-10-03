#ifndef POWERPACK_H
#define POWERPACK_H

#include <vector>

#include "Sensor.h"
#include "PWM.h"
#include "ControlModule.h"
#include "ReferenceGoverner.h"
#include "DatabaseConfig.h"

class Powerpack {
    public:
        Powerpack(int n_pos_channel, int n_neg_channel, std::vector<double> pos_pid_gains, std::vector<double> neg_pid_gains);
        ~Powerpack();

        void get_pos_powerpack_info();
        void get_neg_powerpack_info();
        void print_powerpack_info();  

        void calculate_control_signal();
        void set_all_control_signal();
        void print_control_signal();
        
        void update_sensor();
        void update_reference();
        void update_pwm();

        void run();

    private:
        std::unique_ptr<Sensor> sensor;
        std::unique_ptr<PWM> pwm;
        std::unique_ptr<ReferenceGoverner> referencegoverner;


        int n_pos_channel;
        int n_neg_channel;
        std::vector<ControlModule> modules;
        

};


#endif //POWERPACK_H