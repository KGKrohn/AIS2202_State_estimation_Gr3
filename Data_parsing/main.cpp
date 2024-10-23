#include <rapidcsv.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "DataStore.hpp"
#include "Calibration.hpp"
#include "Vibration_Acc.hpp"
#include "Baseline_wrench.hpp"
#include "Baseline_orientations.hpp"



Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    Baseline_wrench baseline("C:/Users/joelo/AIS2202_code/Del2/AIS2202_State_estimation_Gr3/Data/1-baseline_wrench.csv");
    Baseline_orientations baseorient("C:/Users/joelo/AIS2202_code/Del2/AIS2202_State_estimation_Gr3/Data/1-baseline_orientations.csv");
    int sizeOfOrientVec = baseorient.getSingleTypeColumn_r11_().size();
    std::cout << "Orientation: " << sizeOfOrientVec << std::endl;

    baseline.setVectorMapping(sizeOfOrientVec);
    std::cout << "Wrench: " << baseline.getSingleTypeColumnMapped_fx_().size() << std::endl;




    return 0;
}
