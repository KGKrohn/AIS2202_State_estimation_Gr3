#include "estimation/kalman_filter.h"
#include "../Data_parsing/Baseline_orientations.hpp"
#include "../Data_parsing/Baseline_wrench.hpp"
#include "../Data_parsing/BaselineAcc.hpp"
#include "Eigen/Dense"

int main()
{
    Eigen::MatrixXd P0(9,9);
    Eigen::MatrixXd Tb(3,1);
    Eigen::MatrixXd Fb(3,1);
    Eigen::MatrixXd IMUb(3,1);
    Eigen::MatrixXd r(3,1);
    Eigen::MatrixXd g(3,1);
    Eigen::MatrixXd g_prev(3,1);
    Eigen::MatrixXd x0(9,1);

    Tb << 0.432449, -0.692162, -0.156746;
    Fb << 9.07633, -1.01814,9.98482;
    IMUb << -0.00366194,0.00884945,0.0771078;
    float m =  0.932296;
    r << 0.000279925,5.43937e-05,0.0438988;

    float SDk = 0.5;
    float ff  = 698.3;
    float fr  = 100.2;
    float fa  = 254.3;
    float Sa = 100.0;
    float Sf = 250.0;
    float St = 5000.0;
    std::vector<float> SaSDa= {0.4193, 0.1387, 0.9815};
    std::vector<float> SfSDf= {0.3090, 0.1110, 1.4084};
    std::vector<float> StSDt= {0.0068, 0.0175, 0.0003};

    Baseline_orientations b_o("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/1-baseline_orientations.csv");
    std::vector<float> r11 = b_o.getSingleTypeColumn_r11_();
    std::vector<float> r12 = b_o.getSingleTypeColumn_r12_();
    std::vector<float> r13 = b_o.getSingleTypeColumn_r13_();
    std::vector<float> r21 = b_o.getSingleTypeColumn_r21_();
    std::vector<float> r22 = b_o.getSingleTypeColumn_r22_();
    std::vector<float> r23 = b_o.getSingleTypeColumn_r23_();
    std::vector<float> r31 = b_o.getSingleTypeColumn_r31_();
    std::vector<float> r32 = b_o.getSingleTypeColumn_r32_();
    std::vector<float> r33 = b_o.getSingleTypeColumn_r33_();


    Baseline_wrench b_w("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/1-Baseline_wrench.csv");
    std::vector<float> fx = b_w.getSingleTypeColumn_fx_();
    std::vector<float> fy = b_w.getSingleTypeColumn_fy_();
    std::vector<float> fz = b_w.getSingleTypeColumn_fz_();
    std::vector<float> tx = b_w.getSingleTypeColumn_tx_();
    std::vector<float> ty = b_w.getSingleTypeColumn_ty_();
    std::vector<float> tz = b_w.getSingleTypeColumn_tz_();

    BaselineAcc b_a("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/1-baseline_accel.csv");
    std::vector<float> ax = b_a.getSingleTypeColumn_ax();
    std::vector<float> ay = b_a.getSingleTypeColumn_ay();
    std::vector<float> az = b_a.getSingleTypeColumn_az();

    estimation::kalman_filter kf(x0, P0,m,r);
    kf.update_static_variables(SaSDa, SfSDf, StSDt);
    kf.Q_calc(SDk);
    kf.B_calc();

    auto Rws = kf.Rws(r11[1],r12[1],r13[1],r21[1],r22[1],r23[1],r31[1],r32[1],r33[1]);
    Eigen::MatrixXd g_delta(3,1);
    g_delta << 1,1,1;

    Eigen::MatrixXd u(3,1);
    u = kf.uk(Rws, g_delta, fr, ff, fa);
    kf.predict(u);
    auto x = kf.get_state();
    auto p = kf.get_covariance();
    std::cout << "x: "<< x << std::endl;
    std::cout << "p: "<< p << std::endl;
    kf.update();

}
