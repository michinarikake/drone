#include "matrix.hpp"
#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>
#include <cstdlib>
#include <cmath>

template<typename Type>
    Matrix<Type>::Matrix(int m, int n, Type d)
    {
        std::vector<std::vector<Type> > vv(m, std::vector<Type>(n,d));
        value = vv;
    }
    
    template<typename Type>
    Matrix<Type>::Matrix(Type *array, int m, int n)
    {
        std::vector<std::vector<Type> > vv(m, std::vector<Type>(n,0));
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                vv[i][j] = array[i][j];
            }
        }
        value = vv;
    }

    template<typename Type>
    Matrix<Type>::Matrix(std::vector<std::vector<Type> > val)
    {this->value = val;}

    template<typename Type>
    Matrix<Type>::Matrix(const Matrix<Type>& other_matrix)
    {this->value = other_matrix.get_value();}

    template<typename Type>
    Matrix<Type>::~Matrix()
    {}

    template <typename Type>
    void Matrix<Type>::show()
    {
        unsigned long m = value.size();
        unsigned long n = value[0].size();

        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                std::cout << value[i][j] << " ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    template <typename Type>
    Matrix<Type> Matrix<Type>::transpose()
    {
        unsigned long m = value.size();
        unsigned long n = value[0].size();
        
        std::vector<std::vector<Type> > value_transposed(n, std::vector<Type>(m));
        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                value_transposed[j][i] = value[i][j];
            }
        }
        return Matrix(value_transposed);
    }

    template <typename Type>
    Matrix<Type> Matrix<Type>::inverse()
    {
        unsigned long m = value.size();
        unsigned long n = value[0].size();
        assert(m == n);

        //吐き出し用行列を作る
        std::vector<std::vector<Type> > sweep(m,std::vector<Type>(2*m));
        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < 2*(int)m; j++)
            {
                sweep[i][j] = (i == j-(int)m)? 1 : 0;
                if (j < (int)m)  sweep[i][j] = value[i][j];
            }  
        } 

        //ガウスジョルダン
        for (int i = 0; i < (int)m; i++)
        {
            //最初の数が最大の行を探す
            Type max = sweep[i][i];
            int max_k = i;
            for (int k = i+1; k < (int)m; k++)
            {
                if (std::abs(sweep[k][i]) > std::abs(max)) 
                {
                    max = sweep[k][i];
                    max_k = k;
                }   
            }

            //行の交換
            if (i != max_k) 
            {
                for (int j = 0; j < 2*(int)m; j++) 
                {
                    double tmp = sweep[max_k][j];
                    sweep[max_k][j] = sweep[i][j];
                    sweep[i][j] = tmp;
                }
            }

            //もし正則でないなら正則にする
            if (max == 0)   sweep[i][i] = 0.0001;

            //最初の数で割って行の頭を1に
            Type a = sweep[i][i];
            for (int j = 0; j < 2*(int)m; j++)
            {
                sweep[i][j] /= a;
            }

            //注目している行を引いて他の行の頭を0に
            for (int k = 0; k < (int)m; k++)
            {
                if (i == k) continue;
                Type a = sweep[k][i];
                for (int j = 0; j < 2*(int)m; j++)   
                {
                    sweep[k][j] -= sweep[i][j] * a;
                }
            }
        }

        //逆行列を取り出す
        std::vector<std::vector<Type> > inv(m,std::vector<Type>(m));
        for (int i = 0; i < (int)m; i++) 
        {
            for (int j = 0; j < (int)m; j++) 
            {
                inv[i][j] = sweep[i][m + j];
            }
        }
        return Matrix(inv);
    }

    template<typename Type>
    std::vector<std::vector<Type> > Matrix<Type>::get_value() const
    {
        return value;
    }

    template <typename Type>
    Matrix<Type>& Matrix<Type>::operator=(const Matrix<Type>& other_matrix)
    {
        std::vector<std::vector<Type> > other_value;
        other_value = other_matrix.get_value();
        
        unsigned long m = value.size();
        unsigned long n = value[0].size();

        assert(m == other_value.size());
        assert(n == other_value[0].size());

        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                this->value[i][j] = other_value[i][j];
            }
        }
        return (*this);
    }

    template <typename Type>
    Matrix<Type> Matrix<Type>::operator+(const Matrix<Type>& other_matrix) const
    {
        std::vector<std::vector<Type> > other_value;
        other_value = other_matrix.get_value();
        
        unsigned long m = value.size();
        unsigned long n = value[0].size();

        assert(m == other_value.size());
        assert(n == other_value[0].size());

        std::vector<std::vector<Type> > res(m, std::vector<Type>(n));

        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                res[i][j] = value[i][j] + other_value[i][j];
            }
        }
        return Matrix(res);
    }

    template<typename Type>
    Matrix<Type> Matrix<Type>::operator-(const Matrix<Type>& other_matrix) const
    {
        std::vector<std::vector<Type> > other_value;
        other_value = other_matrix.get_value();
        
        unsigned long m = value.size();
        unsigned long n = value[0].size();

        assert(m == other_value.size());
        assert(n == other_value[0].size());

        std::vector<std::vector<Type> > res(m, std::vector<Type>(n));

        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                res[i][j] = value[i][j] - other_value[i][j];
            }
        }
        return Matrix(res);
    }

    template<typename Type>
    Matrix<Type> Matrix<Type>::operator*(const Matrix<Type>& other_matrix) const
    {
        std::vector<std::vector<Type> > other_value;
        other_value = other_matrix.get_value();

        unsigned long m = value.size();
        unsigned long n = value[0].size();
        unsigned long l = other_value[0].size();

        assert(n == other_value.size());

        std::vector<std::vector<Type> > res(m, std::vector<Type>(l));
        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)l; j++)
            {
                for (int k = 0; k < (int)n; k++)
                {
                    res[i][j] += value[i][k] * other_value[k][j];
                }
            }
        }
        return Matrix(res);
    }

    template<typename Type>
    Type& Matrix<Type>::operator()(int x, int y)
    {
        return this->value[x][y];
    }

    template<typename Type>
    Type& Matrix<Type>::operator()(int x)
    {
        assert(value.size() == 1 or value[0].size() == 1);
        if (value.size() == 1)  return this->value[0][x];
        else    return this->value[x][0];
    }


    // template<typename Type>
    // Matrix<Type> Matrix<Type>::operator[](int x)
    // {
    //     assert(x < value.size());
    //     std::vector<Type> v(1,value[x]);
    //     return Matrix(v);
    // }
