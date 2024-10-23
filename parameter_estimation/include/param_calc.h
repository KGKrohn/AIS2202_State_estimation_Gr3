#ifndef AIS4104_ASSIGNMENTS_PARAM_CALC_H
#define AIS4104_ASSIGNMENTS_PARAM_CALC_H
#include "Eigen/Dense"
#include "vector"
#include <numeric>

class parameter_calculation
{
public:
    parameter_calculation():F(72,1),G(72,1),R(72,3),T(72,1), A(72,3),Ac(72,1){}

    void update_Matrix(std::vector<std::vector<float>> f,
                       std::vector<std::vector<float>> g,
                       std::vector<std::vector<float>> t,
                       std::vector<std::vector<float>> a,
                       std::vector<std::vector<float>> r1,
                       std::vector<std::vector<float>> r2,
                       std::vector<std::vector<float>> r3)
    {
        F = conv_vector_to_72x1_matrix(f);
        Ac = conv_vector_to_72x1_matrix(a);
        G = conv_vector_to_72x1_matrix(g);
        T = conv_vector_to_72x1_matrix(t);
        A = define_A(g);
        R = update_R(r1,r2,r3);
    }
    Eigen::MatrixXd update_R(std::vector<std::vector<float>> R1,
                             std::vector<std::vector<float>> R2,
                             std::vector<std::vector<float>> R3)
    {
        for (int i = 0; i < 24; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                R(i*3 ,j) = R1[i][j];
                R(i*3+1 ,j) = R2[i][j];
                R(i*3+2 ,j) = R3[i][j];
            }
        }
        return R;
    }

    static Eigen::MatrixXd conv_vector_to_72x1_matrix(std::vector<std::vector<float>> vector)
    {
        Eigen::MatrixXd Matrix(72, 1);
        for (int i = 0; i < 24; i++)
        {
            Matrix(i*3, 0) = vector[i][0];
            Matrix(i*3+1, 0) = vector[i][1];
            Matrix(i*3+2, 0) = vector[i][2];
        }
        return Matrix;
    }

    Eigen::MatrixXd f_bias() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd Fb(3,1);
        std::vector<float> fb_x;
        std::vector<float> fb_y;
        std::vector<float> fb_z;

        for (int i = 0; i < 24; i++)
        {
            fb_x.push_back(F(i*3+0));
            fb_y.push_back(F(i*3+1));
            fb_z.push_back(F(i*3+2));
        }

        double fb_x_mean = std::accumulate(fb_x.begin(), fb_x.end(), 0.0) / fb_x.size();
        double fb_y_mean = std::accumulate(fb_y.begin(), fb_y.end(), 0.0) / fb_y.size();
        double fb_z_mean = std::accumulate(fb_z.begin(), fb_z.end(), 0.0) / fb_z.size();
        Fb << fb_x_mean, fb_y_mean, fb_z_mean;

        return Fb;
    }
    Eigen::MatrixXd imu_bias() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd IMUb(3,1);
        std::vector<float> IMUb_x;
        std::vector<float> IMUb_y;
        std::vector<float> IMUb_z;

        for (int i = 0; i < 24; i++)
        {
            IMUb_x.push_back(Ac(i*3+0));
            IMUb_y.push_back(Ac(i*3+1));
            IMUb_z.push_back(Ac(i*3+2));
        }

        double fb_x_mean = std::accumulate(IMUb_x.begin(), IMUb_x.end(), 0.0) / IMUb_x.size();
        double fb_y_mean = std::accumulate(IMUb_y.begin(), IMUb_y.end(), 0.0) / IMUb_y.size();
        double fb_z_mean = std::accumulate(IMUb_z.begin(), IMUb_z.end(), 0.0) / IMUb_z.size();
        IMUb << fb_x_mean, fb_y_mean, fb_z_mean;
        return IMUb;
    }

    std::vector<float> calc_tb_values(int from, int to ,int index_t)
    {
        std::vector<float> tb;
        for (int i = from; i < to; i++)//Clac tx bias
        {
            tb.push_back(T(i*3+index_t));
        }
        return tb;
    }

    Eigen::MatrixXd t_bias_vector() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd Tb(3,1);
        std::vector<float> tb_x;
        std::vector<float> tb_y;
        std::vector<float> tb_z;
        tb_x = calc_tb_values(0,8,0);
        tb_y = calc_tb_values(8,16,1);
        tb_z = calc_tb_values(16,24,2);

        double tb_x_mean = std::accumulate(tb_x.begin(), tb_x.end(), 0.0) / tb_x.size();
        double tb_y_mean = std::accumulate(tb_y.begin(), tb_y.end(), 0.0) / tb_y.size();
        double tb_z_mean = std::accumulate(tb_z.begin(), tb_z.end(), 0.0) / tb_z.size();
        Tb << tb_x_mean, tb_y_mean, tb_z_mean;
        return Tb;
    }


    [[nodiscard]] Eigen::MatrixXd  define_A(std::vector<std::vector<float>> g)
    {
        for (int i = 0; i < 24; i++)
        {
            float gx = g[i][0]; // float gx
            float gy = g[i][1]; // float gy
            float gz = g[i][2]; // float gz

            A(i*3  , 0) = 0;
            A(i*3 , 1) = gz;
            A(i*3 , 2) = -gy;
            A(i*3+ 1 , 0) = -gz;
            A(i*3+ 1 , 1) = 0;
            A(i*3+ 1 , 2) = gx;
            A(i*3+2 , 0) = gy;
            A(i*3+2 , 1) = -gx;
            A(i*3+2 , 2) = 0;
        }
        return A;
    }

    float m_calc()
    {
        Eigen::VectorXd Fv(Eigen::Map<Eigen::VectorXd>(F.data(), F.cols() * F.rows()));
        Eigen::VectorXd Gv(Eigen::Map<Eigen::VectorXd>(G.data(), G.cols() * G.rows()));
        float v1 = Gv.transpose().dot(Fv);
        float v2 = Gv.transpose().dot(Gv);

        float m =v1/v2;
        return m;
    }

    Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
    {
        return A.completeOrthogonalDecomposition().pseudoInverse();
    }

    [[nodiscard]] Eigen::MatrixXd  r_calc()
    {
        float m = m_calc();
        auto r = 1 / m * pseudo_inverse(A) * T;

        return r;
    }

    [[nodiscard]] Eigen::MatrixXd  fg_calc()
    {
        Eigen::MatrixXd f(6,1);
        Eigen::VectorXd f1(3,1);
        Eigen::VectorXd f2(3,1);
        Eigen::Vector3d  r = r_calc();
        float m = m_calc();
        Eigen::Vector3d u1;
        Eigen::Vector3d u2;

        u1 =  R.transpose()*G;
        f1 = m*u1;


        u2 = r.cross(u2);
        f2 = m*u2;

        f << f1(0,0),f1(1,0),f1(2,0), f2(0,0),f2(1,0),f2(2,0);
        return f;
    }

    Eigen::MatrixXd f_bias_vector() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd Fb(3,1);
        std::vector<float> fb_x;
        std::vector<float> fb_y;
        std::vector<float> fb_z;
        fb_x = calc_fb_values(0);
        fb_y = calc_fb_values(1);
        fb_z = calc_fb_values(2);
        double fb_x_mean = std::accumulate(fb_x.begin(), fb_x.end(), 0.0) / fb_x.size();
        double fb_y_mean = std::accumulate(fb_y.begin(), fb_y.end(), 0.0) / fb_y.size();
        double fb_z_mean = std::accumulate(fb_z.begin(), fb_z.end(), 0.0) / fb_z.size();
        Fb << fb_x_mean, fb_y_mean, fb_z_mean;
        return Fb;
    }

    Eigen::MatrixXd calc_fb_matrix() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd Fb_cal(72,1);
        Fb_cal = F;
        for (int i = 0; i < 23; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if((G(i*3 + j)> 9) and (G(i*3 + j+3) < -9))
                {
                    float fb_bias = ((F(i*3 + j) + F(i*3 + j+3))/2.0);
                    Fb_cal(i*3 + j) = Fb_cal(i*3 + j)  - fb_bias;
                    Fb_cal(i*3 + j+3) =Fb_cal(i*3 + j+3) + fb_bias;
                }
                if((G(i*3+j) < -9) and (G(i*3+j+3)> 9))
                {
                    float fb_bias = ((F(i*3 + j) + F(i*3 + j+3))/2.0);
                    Fb_cal(i*3 + j) = Fb_cal(i*3 + j)  + fb_bias;
                    Fb_cal(i*3 + j+3) =Fb_cal(i*3 + j+3) - fb_bias;
                }
            }
        }
        return Fb_cal;
    }
    std::vector<float> calc_fb_values(int index_f1) // calculate the bias in the force sensor.
    {
        std::vector<float> fb_bias;
        for (int i = 0; i < 23; i++)
        {
            if((G(i*3 + index_f1)> 9) and (G(i*3 + index_f1+3) < -9))
            {
                fb_bias.push_back((F(i*3 + index_f1) + F(i*3 + index_f1+3))/2.0);
            }
            if((G(i*3+index_f1) < -9) and (G(i*3+index_f1+3)> 9))
            {
                fb_bias.push_back((F(i*3+index_f1) + F(i*3+index_f1+3))/2.0);
            }

        }
        return fb_bias;
    }
public:
    Eigen::MatrixXd F;
    Eigen::MatrixXd G;
    Eigen::MatrixXd T;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
    Eigen::MatrixXd Ac;
};



#endif //AIS4104_ASSIGNMENTS_PARAM_CALC_H
