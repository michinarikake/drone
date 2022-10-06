#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>
#include <cstdlib>
#include <cmath>
#include "kalman.hpp"

Kalman::Kalman(double sampling_interval)
{
    dt = sampling_interval;
    g_vector =  Matrix<double>(std::vector<std::vector<double> >({{0.0},{0.0},{9.80655}}));
    // m_vectorは後で初期化
    m_vector = Matrix<double>(std::vector<std::vector<double> >({{1.0},{0.0},{1.0}}));

    //変数
    x_estimated_vector = Matrix<double>(std::vector<std::vector<double> >({{1.0},{0.0},{0.0},{0.0},{0.01},{0.01},{0.01},{0.0},{0.0},{0.0}}));

    //カルマンフィルタ推定用行列
    for (int i = 0; i < 10; i++)    
    {
        covariance_estimated_matrix(i,i) = 0.01;
    }

    //ノイズモデリング用行列
    // noise_system_matrix{(Matrix<double>(3,3,0))}は後で初期化
    // noise_observation_matrix{(Matrix<double>(6,6,0))}は後で初期化
    noise_system_matrix = 
        Matrix<double>(std::vector<std::vector<double> >({
            {1.0,0.0,0.0},
            {0.0,1.0,0.0},
            {0.0,0.0,1.0}}));
    noise_observation_matrix = 
        Matrix<double>(std::vector<std::vector<double> >({
            {1.0,0.0,0.0,0.0,0.0,0.0},
            {0.0,1.0,0.0,0.0,0.0,0.0},
            {0.0,0.0,1.0,0.0,0.0,0.0},
            {0.0,0.0,0.0,1.0,0.0,0.0},
            {0.0,0.0,0.0,0.0,1.0,0.0},
            {0.0,0.0,0.0,0.0,0.0,1.0}}));
    noise_omega_matrix = 
        Matrix<double>(std::vector<std::vector<double> >({
            {0.0,0.0,0.0},
            {0.0,0.0,0.0},
            {0.0,0.0,0.0},
            {0.0,0.0,0.0},
            {dt,0.0,0.0},
            {0.0,dt,0.0},
            {0.0,0.0,dt},
            {0.0,0.0,0.0},
            {0.0,0.0,0.0},
            {0.0,0.0,0.0}}));

    //磁気誤差の状態方程式用の行列（経験式）
    magnetic_coefficient_matrix = 
        Matrix<double>(std::vector<std::vector<double> >({
            {-0.0033,0.0,0.0},
            {0.0,-0.0032,0.0},
            {0.0,0.0,-0.0036}}));
}

Kalman::~Kalman()
{
    std::cout << "goodbye";
}

void Kalman::normalize_x_estimated_vector()
{
    double norm;
    for(int i = 0; i < 4; i++)
    {
        norm += x_estimated_vector(i)* x_estimated_vector(i);
    }
    norm = sqrt(norm);
    if (norm == 0)   return;
    for(int i = 0; i < 4; i++)
    {
        x_estimated_vector(i) /= norm;
    }
}

void Kalman::normalize_x_vector()
{
    double norm;
    for(int i = 0; i < 4; i++)
    {
        norm += x_vector(i)* x_vector(i);
    }
    norm = sqrt(norm);
    if (norm == 0)   return;
    for(int i = 0; i < 4; i++)
    {
        x_vector(i) /= norm;
    }
}

/*
観測方程式
引数：x_estimated_vector　事前推定値
返り値：yの推定値
*/
Matrix<double> Kalman::observation_eq(Matrix<double>& x_estimated_vector)
{
    Matrix<double> rotation_matrix(3,3,0.0);
    rotation_matrix(0,0) = 
    x_estimated_vector(0) * x_estimated_vector(0) 
    + x_estimated_vector(1) * x_estimated_vector(1) 
    - x_estimated_vector(2) * x_estimated_vector(2) 
    - x_estimated_vector(3) * x_estimated_vector(3);

    rotation_matrix(0,1) = 
    2.0 * (x_estimated_vector(1) * x_estimated_vector(2) 
    + x_estimated_vector(0) * x_estimated_vector(3));

    rotation_matrix(0,2) = 
    2.0 * (x_estimated_vector(3) * x_estimated_vector(1) 
    - x_estimated_vector(2) * x_estimated_vector(0));

    rotation_matrix(1,0) = 
    2.0 * (x_estimated_vector(1) * x_estimated_vector(2) 
    - x_estimated_vector(3) * x_estimated_vector(0));

    rotation_matrix(1,1) = 
    x_estimated_vector(0) * x_estimated_vector(0) 
    - x_estimated_vector(1) * x_estimated_vector(1) 
    + x_estimated_vector(2) * x_estimated_vector(2) 
    - x_estimated_vector(3) * x_estimated_vector(3);

    rotation_matrix(1,2) = 
    2.0 * (x_estimated_vector(2) * x_estimated_vector(3) 
    + x_estimated_vector(1) * x_estimated_vector(0));

    rotation_matrix(2,0) = 
    2.0 * (x_estimated_vector(3) * x_estimated_vector(1) 
    + x_estimated_vector(2) * x_estimated_vector(0));

    rotation_matrix(2,1) =
    2.0 * (x_estimated_vector(2) * x_estimated_vector(3) 
    - x_estimated_vector(1) * x_estimated_vector(0));

    rotation_matrix(2,2) = 
    x_estimated_vector(0) * x_estimated_vector(0) 
    - x_estimated_vector(2) * x_estimated_vector(2) 
    + x_estimated_vector(3) * x_estimated_vector(3) 
    - x_estimated_vector(1) * x_estimated_vector(1);

    Matrix<double> m{Matrix<double>({
    {
        rotation_matrix(0,2) * g_vector(2) 
        + rotation_matrix(0,0) * x_estimated_vector(7) 
        + rotation_matrix(0,1) * x_estimated_vector(8) 
        + rotation_matrix(0,2) * x_estimated_vector(9)},

    {
        rotation_matrix(1,2) * g_vector(2) 
        + rotation_matrix(1,0) * x_estimated_vector(7) 
        + rotation_matrix(1,1) * x_estimated_vector(8) 
        + rotation_matrix(1,2) * x_estimated_vector(9)},

    {
        rotation_matrix(2,2) * g_vector(2) 
        + rotation_matrix(2,0) * x_estimated_vector(7) 
        + rotation_matrix(2,1) * x_estimated_vector(8) 
        + rotation_matrix(2,2) * x_estimated_vector(9)},

    {
        rotation_matrix(0,0) * m_vector(0) 
        + rotation_matrix(0,1) * m_vector(1) 
        + rotation_matrix(0,2) * m_vector(2)},

    {
        rotation_matrix(1,0) * m_vector(0) 
        + rotation_matrix(1,1) * m_vector(1) 
        + rotation_matrix(1,2) * m_vector(2)},

    {
        rotation_matrix(2,0) * m_vector(0) 
        + rotation_matrix(2,1) * m_vector(1) 
        + rotation_matrix(2,2) * m_vector(2)}
    })};

    return m;
}

/* 
事前推定値を求めるための関数
引数：omega_measured_vector ジャイロセンサーの計測値[w_x,w_y,w_z]
返り値：none
*/
void Kalman::estimate_x_estimated_vector(Matrix<double> omega_measured_vector)
{
    // std::cout << "start estimate_x_estimated_vector" << "\n";
    x_estimated_vector = Matrix<double>({
    {
        (- 0.5 * dt * x_vector(1) * (omega_measured_vector(0) - x_vector(4)) 
        - 0.5 * dt * x_vector(2) * (omega_measured_vector(1) - x_vector(5)) 
        - 0.5 * dt * x_vector(3) * (omega_measured_vector(2) - x_vector(6)) 
        + x_vector(0) )}, 

    {
        (0.5 * dt * x_vector(0) * (omega_measured_vector(0) - x_vector(4)) 
        - 0.5 * dt * x_vector(3) * (omega_measured_vector(1) - x_vector(5)) 
        + 0.5 * dt * x_vector(2) * (omega_measured_vector(2) - x_vector(6)) 
        + x_vector(1) )}, 
    {
        (0.5 * dt * x_vector(3) * (omega_measured_vector(0) - x_vector(4))  
        + 0.5 * dt * x_vector(0) * (omega_measured_vector(1) - x_vector(5)) 
        - 0.5 * dt * x_vector(1) * (omega_measured_vector(2) - x_vector(6)) 
        + x_vector(2))}, 
    {
        (- 0.5 * dt * x_vector(2) * (omega_measured_vector(0) - x_vector(4)) 
        + 0.5 * dt * x_vector(1) * (omega_measured_vector(1) - x_vector(5)) 
        + 0.5 * dt * x_vector(0) * (omega_measured_vector(2) - x_vector(6)) 
        + x_vector(3))}, 
    {
        x_vector(4) * (1 - magnetic_coefficient_matrix(0,0) * dt)},
    {
        x_vector(5) * (1 - magnetic_coefficient_matrix(1,1) * dt)},
    {
        x_vector(6) * (1 - magnetic_coefficient_matrix(2,2) * dt)},
    {
        x_vector(7)},
    {
        x_vector(8)},
    {
        x_vector(9)}
    });

    normalize_x_estimated_vector();
    // std::cout << "finish estimate_x_estimated_vector" << "\n";
}

/*
事前共分散行列を求めるための関数
引数：none
返り値：none
*/
void Kalman::estimate_covariance_estimated_matrix()
{
    // std::cout << "start estimate_covariance_estimated_matrix" << "\n";
    // std::cout << "state_eq_matrix" << "\n";
    // state_eq_matrix.show();
    // std::cout << "covariance_matrix" << "\n";
    // covariance_matrix.show();
    // std::cout << "state_eq_matrix.transpose()" << "\n";
    // state_eq_matrix.transpose().show();
    // std::cout << "noise_omega_matrix" << "\n";
    // noise_omega_matrix.show();
    // std::cout << "noise_system_matrix " << "\n";
    // noise_system_matrix.show();

    covariance_estimated_matrix = 
    state_eq_matrix * covariance_matrix * state_eq_matrix.transpose() 
    + noise_omega_matrix * noise_system_matrix * noise_omega_matrix.transpose();
    // std::cout << "finish etimate_covariance_estimated_matrix" << "\n";
}

/*
カルマンゲインを更新するための関数
引数：none
返り値：none
*/
void Kalman::update_kalman_gain_matrix()
{
    std::cout << "start update_kalman_gain_matrix" << "\n";
    std::cout << "covariance_matrix" << "\n";
    covariance_matrix.show();
    std::cout << "observation_eq_matrix" << "\n";
    observation_eq_matrix.show();
    std::cout << "covariance_estimated_matrix" << "\n";
    covariance_estimated_matrix.show();
    std::cout << "noise_observation_matrix" << "\n";
    noise_observation_matrix.show();
    (observation_eq_matrix * covariance_estimated_matrix).show();
    std::cout << "6*6行列" << "\n";
    (observation_eq_matrix * covariance_estimated_matrix * observation_eq_matrix.transpose() 
    + noise_observation_matrix).inverse().show();

    kalman_gain_matrix =
    (covariance_matrix 
    * observation_eq_matrix.transpose() 
    * (observation_eq_matrix * covariance_estimated_matrix * observation_eq_matrix.transpose() 
    + noise_observation_matrix).inverse());
    // std::cout << "finish update_kalman_gain_matrix" << "\n";
}

/*
事後推定値を更新するための関数
引数：y_measured_vector 加速度計と磁気センサの計測値[a_x,a_y,a_z,m_x,m_y,m_z]
返り値：none
*/
void Kalman::update_x_vector(Matrix<double> y_measured_vector)
{
    // std::cout << "start update_x_vector" << "\n";
    // std::cout << "x_vector" << "\n";
    // x_estimated_vector.show();
    // std::cout << "kalman_gain_matrix" << "\n";
    // kalman_gain_matrix.show();
    // std::cout << "y_measured_vector" << "\n";
    // y_measured_vector.show();
    // std::cout << "x_estimated_vector" << "\n";
    // x_estimated_vector.show();
    // std::cout << "observation_eq(x_estimated_vector)" << "\n";
    // observation_eq(x_estimated_vector).show();
    // (x_estimated_vector 
    // + kalman_gain_matrix * (y_measured_vector - observation_eq(x_estimated_vector))).show();

    x_vector = 
    x_estimated_vector 
    + kalman_gain_matrix * (y_measured_vector - observation_eq(x_estimated_vector));
    // std::cout << "finish update_x_vector" << "\n";
    normalize_x_vector();
}

/*
事後共分散行列を更新するための関数
引数：none
返り値：none
*/
void Kalman::update_covariance_matrix()
{
    // std::cout << "start update_covariance_matrix" << "\n";
    // std::cout << "covariance_matrix" << "\n";
    // covariance_matrix.show();
    // std::cout << "covariance_estimated_matrixx" << "\n";
    // covariance_estimated_matrix.show();
    // std::cout << "kalman_gain_matrix" << "\n";
    // kalman_gain_matrix.show();
    // std::cout << "observation_eq_matrix " << "\n";
    // observation_eq_matrix .show();
    // std::cout << "covariance_estimated_matrix" << "\n";
    // covariance_estimated_matrix.show();

    covariance_matrix = 
    (covariance_estimated_matrix 
    - kalman_gain_matrix * observation_eq_matrix * covariance_estimated_matrix);
    // std::cout << "finish update_covariance_matrix" << "\n";
}

/*
状態方程式の線形近似表現行列を更新するための関数
引数：omega_measured　ジャイロセンサーの計測値[w_x,w_y,w_z]
返り値：none
*/
void Kalman::update_state_eq_matrix(Matrix<double> omega_measured)
{
    // std::cout << "start update_state_eq_matrix" << "\n";
    state_eq_matrix =  Matrix<double>({
    {
        1.0, 
        -0.5 * (omega_measured(0) - x_estimated_vector(4)) * dt, 
        -0.5 * (omega_measured(1) - x_estimated_vector(5)) * dt, 
        -0.5 * (omega_measured(2) - x_estimated_vector(6)) * dt, 
        0.5 * x_estimated_vector(1) * dt, 
        0.5 * x_estimated_vector(2) * dt, 
        0.5 * x_estimated_vector(3) * dt, 
        0.0, 
        0.0, 
        0.0},
    {
        0.5 * (omega_measured(0) - x_estimated_vector(4)) * dt, 
        1.0, 
        0.5 * (omega_measured(2) - x_estimated_vector(6)) * dt, 
        -0.5 * (omega_measured(1) - x_estimated_vector(5)) * dt, 
        -0.5 * x_estimated_vector(0) * dt,
        0.5 * x_estimated_vector(3) * dt, 
        -0.5 * x_estimated_vector(2) * dt, 
        0.0, 
        0.0, 
        0.0},
    {
        0.5 * (omega_measured(1) - x_estimated_vector(5)) * dt, 
        -0.5 * (omega_measured(2) - x_estimated_vector(6)) * dt, 
        1, 
        0.5 * (omega_measured(0) - x_estimated_vector(4)) * dt, 
        -0.5 * x_estimated_vector(3) * dt, 
        -0.5 * x_estimated_vector(0) * dt, 
        0.5 * x_estimated_vector(1) * dt, 
        0, 
        0, 
        0},
    {
        0.5 * (omega_measured(2) - x_estimated_vector(6)) * dt, 
        0.5 * (omega_measured(1) - x_estimated_vector(5)) * dt, 
        -0.5 * (omega_measured(0) - x_estimated_vector(4)) * dt, 
        1.0, 
        0.5 * x_estimated_vector(2) * dt, 
        -0.5 * x_estimated_vector(1) * dt, 
        -0.5 * x_estimated_vector(0) * dt, 
        0.0, 
        0.0, 
        0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0 - magnetic_coefficient_matrix(0,0) * dt, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0 - magnetic_coefficient_matrix(1,1) * dt, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 - magnetic_coefficient_matrix(2,2) * dt, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    });
    // std::cout << "finish update_state_eq_matrix" << "\n";
}

/*
観測方程式の線形近似表現行列を更新するための関数
引数： y_measured_vector 加速度計と磁気センサの計測値[a_x,a_y,a_z,m_x,m_y,m_z]
返り値：none
*/
void Kalman::update_observation_eq_matrix()
{
    // std::cout << "start update_observation_eq_matrix" << "\n";
    /*
    q = [q1.T, q2.T, q3.T].Tと分解すると（回転行列）、観測方程式は[q1(g+a), q2(g+a), q3(g+a), q1m, q2m, q3m]
    これのヤコビアンを求めたい
    q1をrotation1とする。
    https://tech-blog.abeja.asia/entry/research_drone_ins_202207
    を参考にした
    */
    Matrix<double> rotation1_dot_matrix(3,10,0.0);
    Matrix<double> rotation2_dot_matrix(3,10,0.0);
    Matrix<double> rotation3_dot_matrix(3,10,0.0);
    rotation1_dot_matrix = Matrix<double>({
        {
            2*x_estimated_vector(0), 
            -2*x_estimated_vector(1), 
            -2*x_estimated_vector(2), 
            2*x_estimated_vector(3), 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0},
        {
            -2*x_estimated_vector(3), 
            2*x_estimated_vector(2), 
            2*x_estimated_vector(1), 
            -2*x_estimated_vector(0), 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0,
            0.0},
        {
            2*x_estimated_vector(2), 
            2*x_estimated_vector(3), 
            2*x_estimated_vector(0), 
            2*x_estimated_vector(1), 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0}
    });

    rotation1_dot_matrix = Matrix<double>({
        {2 * x_estimated_vector(3), 
        2 * x_estimated_vector(2), 
        2 * x_estimated_vector(1), 
        2 * x_estimated_vector(0), 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0},
        {2 * x_estimated_vector(0), 
        -2 * x_estimated_vector(1), 
        2 * x_estimated_vector(2), 
        -2 * x_estimated_vector(3), 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0},
        {-2 * x_estimated_vector(1), 
        -2 * x_estimated_vector(0), 
        2 * x_estimated_vector(3), 
        2 * x_estimated_vector(2), 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0}
    });

    rotation1_dot_matrix = Matrix<double>({
        {-2 * x_estimated_vector(2), 
        2 * x_estimated_vector(3), 
        -2 * x_estimated_vector(0), 
        2 * x_estimated_vector(1),
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0},

        {2 * x_estimated_vector(1), 
        2 * x_estimated_vector(0), 
        2 * x_estimated_vector(3), 
        2 * x_estimated_vector(2), 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0},

        {2 * x_estimated_vector(0), 
        -2 * x_estimated_vector(1), 
        -2 * x_estimated_vector(2), 
        2 * x_estimated_vector(3), 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        0.0}
    });
    
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            observation_eq_matrix(i,j) = 
            g_vector(2) * rotation3_dot_matrix(i,j) 
            + x_estimated_vector(7) * rotation1_dot_matrix(i,j) 
            + x_estimated_vector(8) * rotation2_dot_matrix(i,j) 
            + x_estimated_vector(9) * rotation3_dot_matrix(i,j);

            observation_eq_matrix(i+3,j) =
            m_vector(0) * rotation1_dot_matrix(i,j) 
            + m_vector(1) * rotation2_dot_matrix(i,j) 
            + m_vector(2) * rotation3_dot_matrix(i,j);
        }
    }
    // std::cout << "finish update_observation_eq_matrix" << "\n";
}

void Kalman::show_kalman_gain()
{
    kalman_gain_matrix.show();
}

/*
姿勢をクォータニオンとして求める関数
引数：omega_measured_vector ジャイロセンサーの計測値[w_x,w_y,w_z], 
    y_measured_vector 加速度計と磁気センサの計測値[a_x,a_y,a_z,m_x,m_y,m_z]
返り値：quarternion 現在の姿勢を表すクォータニオン
*/
Matrix<double> Kalman::calculate_quarternion(Matrix<double> omega_measured_vector, Matrix<double> y_measured_vector)
{
    std::cout << "start calculate_quarternion" << "\n";
    assert(omega_measured_vector.get_value().size() == 3);
    assert(omega_measured_vector.get_value()[0].size() == 1);
    assert(y_measured_vector.get_value().size() == 6);
    assert(omega_measured_vector.get_value()[0].size() == 1);

    //線形近似用行列の更新
    update_state_eq_matrix(omega_measured_vector);
    update_observation_eq_matrix();

    // //状態の更新
    update_kalman_gain_matrix();
    update_x_vector(y_measured_vector);
    update_covariance_matrix();


    // 次のステップの予測
    estimate_x_estimated_vector(omega_measured_vector);
    estimate_covariance_estimated_matrix();

    Matrix<double> quarternion
    {Matrix<double>(std::vector<std::vector<double> >({{x_vector(0)},{x_vector(1)},{x_vector(2)},{x_vector(3)}}))};
    std::cout << "finish calculate_quarternion" << "\n";
    return quarternion;
}