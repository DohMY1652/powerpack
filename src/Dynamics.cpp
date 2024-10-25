#include "Dynamics.h"

#include <cmath>
#include <iostream>
#include <vector>

#include "DatabaseConfig.h"

Dynamics::Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
      valve_const({0.0002181, 0.007379, 0.7191}) {
    std::vector<double> parameters = databaseconfig->get_channel_volume();
}

Dynamics::~Dynamics() {}

void Dynamics::set_channel(int _channel) { channel = _channel; }

void Dynamics::calculate_valve_dynamic(double input, double pressure_in,
                                       double pressure_out, double volume) {
    double lpm2kgps = 0.0002155;  // for air

    if (input >= 100) {
        input = 100;
    } else if (input <= 0) {
        input = 0;
    }

    if (pressure_in - pressure_out >= 0) {
        flow_rate = (valve_const[0] * pressure_in + valve_const[1] * input -
                     valve_const[2]) *
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in);

        if (flow_rate >= 100) {
            flow_rate = 100;
            round_input = 0;
            round_pressure_in = 0;
            round_pressure_out = 0;
        } else if (flow_rate > 0) {
            round_input = valve_const[1] *
                          sqrt(2 * (pressure_in - pressure_out) * pressure_in);
            round_pressure_in =
                valve_const[0] *
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) +
                (valve_const[0] * pressure_in + valve_const[1] * input -
                 valve_const[2]) /
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) *
                    (4 * (pressure_in - pressure_out));
            round_pressure_out =
                valve_const[0] *
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) +
                (valve_const[0] * pressure_in + valve_const[1] * input -
                 valve_const[2]) /
                    sqrt(2 * (pressure_in - pressure_out) * pressure_in) *
                    (-2 * pressure_in);
        } else {
            flow_rate = 0;
            round_input = 0;
            round_pressure_in = 0;
            round_pressure_out = 0;
        }
    } else {
        flow_rate = 0;
        round_input = 0;
        round_pressure_in = 0;
        round_pressure_out = 0;
    }

    flow_rate *= lpm2kgps;
    round_input *= (gas_constant * temperature / volume);
    round_pressure_in *= (gas_constant * temperature / volume);
    round_pressure_out *= (gas_constant * temperature / volume);
    round_input *= lpm2kgps;
    round_pressure_in *= lpm2kgps;
    round_pressure_out *= lpm2kgps;
}

double Dynamics::get_flow_rate() const { return flow_rate; }
double Dynamics::get_round_input() const { return round_input; }
double Dynamics::get_round_pressure_in() const { return round_pressure_in; }
double Dynamics::get_round_pressure_out() const { return round_pressure_out; }