#ifndef VALVE_H
#define VALVE_H

#include <vector>

class Valve {
    public:
        Valve();
        ~Valve();
        void calculate_valve_dynamic(double input, double pressure_in, double pressure_out);
        double get_flow_rate();
        double get_round_input();
        double get_round_pressure_in();
        double get_round_pressure_out();

    private:
        bool is_positive;
        std::vector<double> valve_const = {0.0002181, 0.007379, 0.7191};
        double flow_rate;
        double round_input;
        double round_pressure_in;
        double round_pressure_out;
        

};


#endif //VALVE_H