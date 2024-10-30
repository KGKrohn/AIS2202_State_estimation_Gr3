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
    std::vector<double> tr = b_o.getSingleTypeColumn_t_();
    std::vector<double> r11 = b_o.getSingleTypeColumn_r11_();
    std::vector<double> r12 = b_o.getSingleTypeColumn_r12_();
    std::vector<double> r13 = b_o.getSingleTypeColumn_r13_();
    std::vector<double> r21 = b_o.getSingleTypeColumn_r21_();
    std::vector<double> r22 = b_o.getSingleTypeColumn_r22_();
    std::vector<double> r23 = b_o.getSingleTypeColumn_r23_();
    std::vector<double> r31 = b_o.getSingleTypeColumn_r31_();
    std::vector<double> r32 = b_o.getSingleTypeColumn_r32_();
    std::vector<double> r33 = b_o.getSingleTypeColumn_r33_();


    Baseline_wrench b_w("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/1-Baseline_wrench.csv");
    std::vector<double> tft = b_w.getSingleTypeColumn_t_();
    std::vector<double> fx = b_w.getSingleTypeColumn_fx_();
    std::vector<double> fy = b_w.getSingleTypeColumn_fy_();
    std::vector<double> fz = b_w.getSingleTypeColumn_fz_();
    std::vector<double> tx = b_w.getSingleTypeColumn_tx_();
    std::vector<double> ty = b_w.getSingleTypeColumn_ty_();
    std::vector<double> tz = b_w.getSingleTypeColumn_tz_();

    BaselineAcc b_a("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/1-baseline_accel.csv");
    std::vector<double> ta = b_a.getSingleTypeColumn_t();
    std::vector<double> ax = b_a.getSingleTypeColumn_ax();
    std::vector<double> ay = b_a.getSingleTypeColumn_ay();
    std::vector<double> az = b_a.getSingleTypeColumn_az();

    estimation::kalman_filter kf(x0, P0,m,r);
    std::vector<double> Ta = kf.scale_time_array(ta);
    std::vector<double> Tft = kf.scale_time_array(tft);
    std::vector<double> Tr = kf.scale_time_array(tr);

    kf.update_static_variables(SaSDa, SfSDf, StSDt,0.5);
    kf.B_calc();
    std::vector<Eigen::VectorXd> X= {};
    std::vector<Eigen::VectorXd> Xf= {};
    int ir = 0;
    int ia = 0;
    int ift = 0;
    kf.write_state_to_csv("Test1.csv",false);
    for (int i = 0; i < Tr.back(); i++)
    {

        if (Tr[ir] == i)
        {
            kf.Rws(r11[ir],r12[ir],r13[ir],r21[ir],r22[ir],r23[ir],r31[ir],r32[ir],r33[ir]);
            ir = ir + 1;
        }
        if (Ta[ia] == i)
        {
            kf.uk(ax[ia],ay[ia],az[ia], fr, ff, fa);
            kf.za_update(ax[ia], ay[ia], az[ia], fx[ift], fy[ift], fz[ift], tx[ift], ty[ift], tz[ift]);
            kf.predict();
            kf.update();
            X.emplace_back(kf.get_state());
            ia = ia + 1;
        }
        if (Tft[ift] == i)
        {
            kf.uk(ax[ia],ay[ia],az[ia], fr, ff, fa);
            kf.zf_update(ax[ia], ay[ia], az[ia], fx[ift], fy[ift], fz[ift], tx[ift], ty[ift], tz[ift]);
            kf.predict();
            kf.update();
            Xf.emplace_back(kf.get_state());
            kf.write_state_to_csv("Test1.csv",true);
            ift = ift + 1;
        }

    }

    for (int i = 0; i < X.size(); i++)
    {

    }
    std::cout << "ir " << ir<< std::endl;
    std::cout << "ia " << ia<< std::endl;
    std::cout << "ift " << ift<< std::endl;
    std::cout << "X.size() " << X.size() << std::endl;
    std::cout << "Xf.size() " << Xf.size() << std::endl;
    std::cout << "X 500 "  << Xf[500] << std::endl;
    std::cout << "X 1000 "  << Xf[1000] << std::endl;
    std::cout << "X 2000 "  << Xf[2000] << std::endl;
    std::cout << "X 3000 "  << Xf[3000] << std::endl;
    std::cout << "X 4000 "  << Xf[4000] << std::endl;

}
