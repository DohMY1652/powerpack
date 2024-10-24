#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include <memory>

#include "Sensor.h"
#include "ReferenceGoverner.h"
#include "DatabaseConfig.h"
#include "Dynamics.h"
#include "QP.h"


class Solver {
public:
    Solver(bool is_positive, std::shared_ptr<Sensor>& sensor, std::shared_ptr<ReferenceGoverner>& referencegoverner, std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Solver();

private:
    std::shared_ptr<Sensor> &sensor;                     
    std::shared_ptr<ReferenceGoverner> &referencegoverner; 
    std::shared_ptr<DatabaseConfig> &databaseconfig;

    int NP;
    int n_x;
    int n_u;
    double Ts;
    double Q;
    double R;
    double ku_micro;
    double ku_macro;
    double ku_atm;

    bool is_positive;
    double P_now;
    double P_micro;
    double P_macro;
    std::vector<double> P_ref;
    std::vector<double> U_ref; 

    std::unique_ptr<Dynamics> dynamics;
    std::unique_ptr<QP> qp;

};


#endif // SOLVER_H