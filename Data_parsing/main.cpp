#include <rapidcsv.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "DataStore.hpp"
#include "Calibration.hpp"



CalibrationFTS fileRead2("D:/AIS2202_code/AIS2202_State_estimation_Gr3/Data/0-calibration_fts-accel.csv");

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    std::cout << "[ " << fileRead2.getForceVector(0)[0] << ", "
    << fileRead2.getForceVector(0)[1] << ", " << fileRead2.getForceVector(0)[2] << " ]" << std::endl;

    return 0;
}
