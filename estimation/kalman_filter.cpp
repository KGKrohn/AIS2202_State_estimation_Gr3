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

    Tb      << 0.432449, -0.692162, -0.156746;      //Sensor bias
    Fb      << 9.07633, -1.01814,9.98482;           //Sensor bias
    IMUb    << -0.00366194,0.00884945,0.0771078;    //Sensor bias
    r       << 0.000279925,5.43937e-05,0.0438988;   // Calculated values
    float m =  0.932296;                            // Calculated values
    //float m =  0.932;                             // From report
    //r << 0.0,0.0,0.044;                           // From report

    float SDk = 0.5;                                //From report
    float ff  = 698.3;                              //From report
    float fr  = 100.2;                              //From report
    float fa  = 254.3;                              //From report
    float Sa = 100.0;
    float Sf = 250.0;
    float St = 5000.0;
    std::vector<float> SaSDa= {0.4193, 0.1387, 0.9815};//From report
    std::vector<float> SfSDf= {0.3090, 0.1110, 1.4084};//From report
    std::vector<float> StSDt= {0.0068, 0.0175, 0.0003};//From report

    //Gather dataset
    //Baseline_orientations b_o("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/2-vibrations_orientations.csv");
    Baseline_orientations b_o("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/3-vibrations-contact_orientations.csv");
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


    //Baseline_wrench b_w("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/2-vibrations_wrench.csv");
    Baseline_wrench b_w("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/3-vibrations-contact_wrench.csv");
    std::vector<double> tft = b_w.getSingleTypeColumn_t_();
    std::vector<double> fx = b_w.getSingleTypeColumn_fx_();
    std::vector<double> fy = b_w.getSingleTypeColumn_fy_();
    std::vector<double> fz = b_w.getSingleTypeColumn_fz_();
    std::vector<double> tx = b_w.getSingleTypeColumn_tx_();
    std::vector<double> ty = b_w.getSingleTypeColumn_ty_();
    std::vector<double> tz = b_w.getSingleTypeColumn_tz_();


    //BaselineAcc b_a("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/2-vibrations_accel.csv");
    BaselineAcc b_a("C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/Data/3-vibrations-contact_accel.csv");
    std::vector<double> ta = b_a.getSingleTypeColumn_t();
    std::vector<double> ax = b_a.getSingleTypeColumn_ax();
    std::vector<double> ay = b_a.getSingleTypeColumn_ay();
    std::vector<double> az = b_a.getSingleTypeColumn_az();


    estimation::kalman_filter kf(x0, P0,m,r);//Initialize kalman filter
    std::vector<double> Ta = kf.scale_time_array(ta, 8416.0);   // Phase shift to match the time of the other datasets
    std::vector<double> Tft = kf.scale_time_array(tft);                   // Normalize time array
    std::vector<double> Tr = kf.scale_time_array(tr);                     // Normalize time array

    kf.update_static_variables(SaSDa, SfSDf, StSDt); //Initialize static variables


    // Define file path
    std::string file_path= "C:/Skulemappe/NTNU_2022-2025/5_Semester2023/AIS2202_Kubernetikk/Modul_2_State estimation/AIS2202_State_estimation_Gr3/estimation/include/";
    std::string datafile = file_path + "3-Test6.csv"; // Define file name

    int ir = 0;
    int ia = 0;
    int ift = 0;
    kf.write_state_to_csv(datafile,false); // Write header to file
    for (int i = 0; i < Tr.back(); i++)
    {
        if (Tr[ir] == i)
        {
            kf.Rws(r11[ir],r12[ir],r13[ir],
                   r21[ir],r22[ir],r23[ir],
                   r31[ir],r32[ir],r33[ir]);
            kf.uk(fr, ff, fa);

            ir = ir + 1;
        }
        if (Ta[ia] == i)
        {
            kf.za_update(ax[ia], ay[ia], az[ia],
                         fx[ift], fy[ift], fz[ift],
                         tx[ift], ty[ift], tz[ift]);
            //kf.zc_update(ax[ia], ay[ia], az[ia],
            //             fx[ift], fy[ift], fz[ift],
            //             tx[ift], ty[ift], tz[ift]);

            kf.predict(SDk,i);
            kf.update();
            kf.write_state_to_csv(datafile,true,i);// IMU kalmanfilter output
            ia = ia + 1;
        }
        if (Tft[ift] == i)
        {
            kf.zf_update(ax[ia], ay[ia], az[ia],
                         fx[ift], fy[ift], fz[ift],
                         tx[ift], ty[ift], tz[ift]);
            //kf.zc_update(ax[ia], ay[ia], az[ia],
            //             fx[ift], fy[ift], fz[ift],
            //             tx[ift], ty[ift], tz[ift]);
            kf.predict(SDk,i);
            kf.update();
            kf.write_state_to_csv(datafile,true,i);// FTS kalmanfilter output
            ift = ift + 1;
        }
    }
}
