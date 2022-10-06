#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "../matrix/matrix.hpp"
#include "../matrix/matrix.cpp"

class Kalman
{
    //定数
    double dt;
    Matrix<double> g_vector{Matrix<double>(3,1,0)}; //3*1
    Matrix<double> m_vector{Matrix<double>(3,1,0)};//3*1

    //変数
    Matrix<double> x_vector{Matrix<double>(10,1,0)}; //10*1事後推定値,[q0,q1,q2,q3,omega1,omega2,omega3,a_dr1,a_dr2,a_dr3]
    Matrix<double> x_estimated_vector{Matrix<double>(10,1,0)}; //10*1事前推定値

    //カルマンフィルタ推定用行列
    Matrix<double> kalman_gain_matrix{(Matrix<double>(10,6,0))}; //10*6
    Matrix<double> covariance_matrix{Matrix<double>(10,10,0)}; //10*10事後共分散行列
    Matrix<double> covariance_estimated_matrix{Matrix<double>(10,10,0)}; //10*10事前共分散行列

    //拡張カルマンフィルタ線形近似用行列
    Matrix<double> state_eq_matrix{Matrix<double>(10,10,0)}; // 10*10
    Matrix<double> observation_eq_matrix{Matrix<double>(6,10,0)}; //6*10

    //ノイズモデリング用行列(定数)
    Matrix<double> noise_system_matrix{(Matrix<double>(3,3,0))}; //3*3 Q
    Matrix<double> noise_observation_matrix{(Matrix<double>(6,6,0))}; //6*6 R
    Matrix<double> noise_omega_matrix{(Matrix<double>(10,3,0))}; //10*3 G

    //磁気誤差の状態方程式用の行列（経験式、定数）
    Matrix<double> magnetic_coefficient_matrix{Matrix<double>(3,3,0)}; //3*3

    //正規化関数
    void normalize_x_vector();
    void normalize_x_estimated_vector();

    //観測関数
    Matrix<double> observation_eq(Matrix<double>& x_estimated_vector);

    //カルマンフィルタ推定用関数
    void estimate_x_estimated_vector(Matrix<double> omega_measured_vector);
    void estimate_covariance_estimated_matrix();

    //カルマンフィルタ更新用関数
    void update_kalman_gain_matrix();
    void update_x_vector(Matrix<double> y_measured_vector);
    void update_covariance_matrix();
    void update_state_eq_matrix(Matrix<double> omega_measured_vector);
    void update_observation_eq_matrix();


    public:
        Kalman(double sampling_interval);
        ~Kalman();
        void init(); //地磁気、ノイズの取得
        void show_kalman_gain();
        Matrix<double> calculate_quarternion(Matrix<double> omega_measured_vector, Matrix<double> y_measured_vector);
};

#endif