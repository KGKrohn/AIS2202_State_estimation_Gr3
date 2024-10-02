#include <rapidcsv.h>

#include "include/Param_calc.h"

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{

    Eigen::MatrixXd F = Parameter_calculation::Define_F(1,2,3);
    Eigen::MatrixXd G = Parameter_calculation::Define_G(1,2,3);
    Eigen::MatrixXd R = Parameter_calculation::Define_R(1,2,3,4,5,6,7,8,9);
    std::cout << R << std::endl;

    return 0;
}
