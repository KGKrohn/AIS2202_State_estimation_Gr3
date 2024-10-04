#include <rapidcsv.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "DataStore.hpp"
#include "Calibration.hpp"



DataStore fileRead1("D:/AIS2202_code/AIS2202_State_estimation_Gr3/Data/0-calibration_fts-accel.csv");
Calibration fileRead2("D:/AIS2202_code/AIS2202_State_estimation_Gr3/Data/0-calibration_fts-accel.csv");

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    std::string name = fileRead1.getColumnName(0);
    std::cout << name << std::endl;
    std::cout << fileRead1.getColumn(2)[0] << std::endl;
    return 0;
}
