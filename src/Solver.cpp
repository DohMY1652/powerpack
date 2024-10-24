#include <iostream>

#include "Solver.h"

Solver::Solver(bool is_positive, std::shared_ptr<Sensor>& sensor, std::shared_ptr<ReferenceGoverner>& referencegoverner, std::shared_ptr<DatabaseConfig>& databaseconfig)
: sensor(sensor), referencegoverner(referencegoverner), databaseconfig(databaseconfig) {
    
    std::vector<double> parameters = databaseconfig->get_MPC_parameters();
    NP = (int)parameters[0];
    n_x = (int)parameters[1];
    n_u = (int)parameters[2];
    Ts = parameters[3];
    Q = parameters[4];
    R = parameters[5];
    ku_micro = parameters[6];
    ku_macro = parameters[7];
    ku_atm = parameters[8];
    
    dynamics = std::make_unique<Dynamics>(databaseconfig);
    qp = std::make_unique<QP>(databaseconfig);

}

Solver::~Solver() {

}

