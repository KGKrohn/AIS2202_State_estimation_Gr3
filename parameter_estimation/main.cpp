#include <rapidcsv.h>

#include "include/param_calc.h"
#include "../Data_parsing/Calibration.hpp"

CalibrationFTS fileRead2("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/0-calibration_fts-accel.csv");
Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    std::vector<std::vector<float>> t = fileRead2.getTorqueVectorColumn();
    std::vector<std::vector<float>> g = fileRead2.getGVectorColumn();
    std::vector<std::vector<float>> f = fileRead2.getForceVectorColumn();
    parameter_calculation calc;
    calc.update_Matrix(f,g,t);
    Eigen::MatrixXd x = calc.define_A(g);
    std::cout << x << std::endl;


    return 0;
}
