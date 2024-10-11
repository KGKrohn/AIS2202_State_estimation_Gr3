#ifndef AIS4104_ASSIGNMENTS_PARAM_CALC_H
#define AIS4104_ASSIGNMENTS_PARAM_CALC_H
#include "Eigen/Dense"
#include "vector"
#include <numeric>

class parameter_calculation
{
public:

    parameter_calculation():F(72,1),G(72,1),R(72,3),T(72,1), A(72,3),F_cal(72,3){}

    void update_Matrix(std::vector<std::vector<float>> f,
                       std::vector<std::vector<float>> g,
                       std::vector<std::vector<float>> t)
    {
        F = conv_vector_to_72x1_matrix(f);
        G = conv_vector_to_72x1_matrix(g);
        T = conv_vector_to_72x1_matrix(t);
        A = define_A(g);
        F_cal = f_calibration(f = f, g = g);
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

    std::vector<std::vector<float>> f_bias(std::vector<std::vector<float>> f,
                                           std::vector<std::vector<float>> g) // calculate the bias in the force sensor.
    {
        std::vector<std::vector<float>> V(24, std::vector<float>(3,0));
        for (int i = 0; i < 23; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if((g[i][j] > 9) and (g[i+1][j] < -9))
                {
                    V[i][j] = (f[i][j]+f[i+1][j])/2;
                    V[i+1][j] = (f[i][j]+f[i+1][j])/2;
                }
                if((g[i][j] < -9) and (g[i+1][j] > 9))
                {
                    V[i][j] = (f[i][j]+f[i+1][j])/2;
                    V[i+1][j] = (f[i][j]+f[i+1][j])/2;
                }
            }
        }
        return V;
    }
    std::vector<float> calc_tb_values(int from, int to,int index_f1, int index_f2)
    {
        std::vector<float> tb;
        for (int i = from; i < to; i++)//Clac tx bias
        {
            if (F(i*3+index_f1)>0 and F(i*3+index_f2)>0)
            {
                tb.push_back(T(i*3)-( T(i*3+index_f1)-T(i*3+index_f2))); // tb =tx - (ty - tz)
            }
            else if(F(i*3+index_f1)<0 and F(i*3+index_f2)<0)
            {
                tb.push_back(T(i * 3) - (-T(i * 3 + index_f1) + T(i * 3 + index_f2))); // tb =tx - (-ty + tz)
            }
        }
        return tb;
    }

    Eigen::MatrixXd t_bias_vector() // calculate the bias in the force sensor.
    {
        Eigen::MatrixXd Tb(3,1);
        std::vector<float> tb_x;
        std::vector<float> tb_y;
        std::vector<float> tb_z;
        tb_x = calc_tb_values(0,8,1,2);
        tb_y = calc_tb_values(0,8,0,2);
        tb_z = calc_tb_values(0,8,0,1);

        double tb_x_mean = std::accumulate(tb_x.begin(), tb_x.end(), 0.0) / tb_x.size();
        double tb_y_mean = std::accumulate(tb_y.begin(), tb_y.end(), 0.0) / tb_y.size();
        double tb_z_mean = std::accumulate(tb_z.begin(), tb_z.end(), 0.0) / tb_z.size();
        Tb << tb_x_mean, tb_y_mean, tb_z_mean;
        return Tb;
    }
    Eigen::MatrixXd  f_calibration(std::vector<std::vector<float>> f,
                                   std::vector<std::vector<float>> g)// using the bias to calculate the calibration matrix
    {

        std::vector<std::vector<float>> Fb = f_bias(f = f, g = g);
        Eigen::MatrixXd Fb_matrix = conv_vector_to_72x1_matrix(Fb);
        F_cal = F - Fb_matrix;
        return F_cal;
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

    void update_R(std::vector<std::vector<float>> R1,
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
    }


    float m_calc()
    {
        Eigen::VectorXd Fv(Eigen::Map<Eigen::VectorXd>(F_cal.data(), F_cal.cols() * F_cal.rows()));
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
        auto r = 1 / m_calc() * pseudo_inverse(A) * T;

        return r;
    }

    [[nodiscard]] Eigen::MatrixXd  fg_calc()
    {
        Eigen::MatrixXd f(6,1);
        Eigen::VectorXd f1(3,1);
        Eigen::VectorXd f2(3,1);
        Eigen::Vector3d  r = r_calc();
        Eigen::Vector3d u1;
        Eigen::Vector3d u2;

        u1 =  R.transpose()*G;
        f1 = m_calc()*u1;

        u2 = r.cross(u2);
        f2 = m_calc()*u2;

        f << f1(0,0),f1(1,0),f1(2,0), f2(0,0),f2(1,0),f2(2,0);
        return f;
    }

private:
    Eigen::MatrixXd F;
    Eigen::MatrixXd F_cal;
    Eigen::MatrixXd G;
    Eigen::MatrixXd T;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;

};



#endif //AIS4104_ASSIGNMENTS_PARAM_CALC_H
