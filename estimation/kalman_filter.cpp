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
    //float m =  0.932296;
    float m =  0.932;
    //r << 0.000279925,5.43937e-05,0.0438988;
    r << 0.0,0.0,0.044;

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
    std::vector<Eigen::VectorXd> X= {};

    for (int i = 0; i < 600; i++)
    {
        kf.Rws(r11[i],r12[i],r13[i],r21[i],r22[i],r23[i],r31[i],r32[i],r33[i]);

        Eigen::MatrixXd u = kf.uk(ax[int(i*2.56)],ay[int(i*2.56)],az[int(i*2.56)], fr, ff, fa);
        kf.predict(u);
        kf.update();
        X.emplace_back(kf.get_state());
    }
    std::cout << "X" << X[300] << std::endl;


}
