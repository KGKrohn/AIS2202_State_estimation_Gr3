#include <rapidcsv.h>

#include "include/param_calc.h"
#include "../Data_parsing/Calibration.hpp"

CalibrationFTS fileRead2("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/0-calibration_fts-accel.csv");


int main()
{
    std::vector<std::vector<float>> t = fileRead2.getTorqueVectorColumn();
    std::vector<std::vector<float>> g = fileRead2.getGVectorColumn();
    std::vector<std::vector<float>> f = fileRead2.getForceVectorColumn();
    std::vector<std::vector<float>> r1 = fileRead2.getR1VectorColumn();
    std::vector<std::vector<float>> r2 = fileRead2.getR2VectorColumn();
    std::vector<std::vector<float>> r3 = fileRead2.getR3VectorColumn();
    parameter_calculation calc;
    calc.update_Matrix(f=f, g=g, t=t);
    calc.update_R(r1,r2,r3);

    std::cout <<"Torque bias: "<< calc.t_bias_vector() << std::endl;
    //std::cout <<"Mass: "<< calc.m_calc() << std::endl;
    //std::cout <<"Center of mass distance : "<< calc.r_calc() << std::endl;
    //std::cout <<"Force_calibration bias: "<< calc.fg_calc() << std::endl;



    return 0;
}
