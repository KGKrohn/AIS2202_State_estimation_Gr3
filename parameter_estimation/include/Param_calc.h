#ifndef AIS4104_ASSIGNMENTS_PARAM_CALC_H
#define AIS4104_ASSIGNMENTS_PARAM_CALC_H
#include "Eigen/Dense"

class Parameter_calculation
{
public:
    Parameter_calculation();

    static Eigen::MatrixXd Define_F(float fx, float fy, float fz)
    {
        Eigen::MatrixXd F(3,1);
        F << fx, fy, fz;

        return F;
    }


    static Eigen::MatrixXd Define_G(float gx, float gy, float gz)
    {
        Eigen::MatrixXd G(3,1);
        G << gx, gy, gz;

        return G;
    }

    static Eigen::MatrixXd Define_R(float r1, float r2, float r3, float r4, float r5, float r6, float r7, float r8, float r9)
    {
        Eigen::MatrixXd R(3,3);
        R<< r1, r2, r3, r4, r5, r6, r7, r8, r9;

        return R;
    }
};



#endif //AIS4104_ASSIGNMENTS_PARAM_CALC_H
