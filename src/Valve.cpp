#include <cmath>

#include "Valve.h"

Valve::Valve() {

}

Valve::~Valve() {

}

void Valve::calculate_valve_dynamic(double input, double pressure_in, double pressure_out) {
    double lpm2kgps = 0.0002155; //for air

    if (input >= 100) {
        input = 100;
    }
    else if (input <= 0) {
        input = 0;
    }

    if (pressure_in - pressure_out >= 0) {
        flow_rate = (valve_const[0] * pressure_in + valve_const[1] * input-valve_const[2]) * 
        sqrt(2 * (pressure_in - pressure_out) * pressure_in);
        if (flow_rate >= 100) {
            flow_rate = 100;
            round_input = 0;
            round_pressure_in = 0;
            round_pressure_out = 0;
        }
        else if (flow_rate > 0) {
            round_input = valve_const[1]*sqrt(2*(pressure_in-pressure_out)*pressure_in);
            round_pressure_in = valve_const[0] * 
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) +
                    (valve_const[0] * pressure_in + valve_const[1] * input - valve_const[2]) / 
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) * 
                    (4 * (pressure_in - pressure_out));
            round_pressure_out = valve_const[0] * 
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) +
                    (valve_const[0] * pressure_in + valve_const[1] * input - valve_const[2]) / 
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) * 
                    (-2*pressure_in);
        }
        else {
            flow_rate = 0;
            round_input = 0;
            round_pressure_in = 0;
            round_pressure_out = 0;
        }
    }
    else {
        flow_rate = 0;
        round_input = 0;
        round_pressure_in = 0;
        round_pressure_out = 0;
    }

    flow_rate *= lpm2kgps;
    round_input *= lpm2kgps;
    round_pressure_in *= lpm2kgps;
    round_pressure_out *= lpm2kgps;
}

double Valve::get_flow_rate(){
    return flow_rate;
}
double Valve::get_round_input() {
    return round_input;
}
double Valve::get_round_pressure_in() {
    return round_pressure_in;
}
double Valve::get_round_pressure_out() {
    return round_pressure_out;
}