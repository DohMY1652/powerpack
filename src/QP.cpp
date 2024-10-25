#include "QP.h"

#include <iostream>
#include <vector>

#include "DatabaseConfig.h"

QP::QP(std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig) {}

QP::~QP() {}
