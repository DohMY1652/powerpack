#include <iostream>
#include <vector>

#include "QP.h"
#include "DatabaseConfig.h"


QP::QP(std::shared_ptr<DatabaseConfig>& databaseconfig) 
 : databaseconfig(databaseconfig) {

}

QP::~QP() {
    
}