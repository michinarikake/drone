#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <vector>

template <typename Type>
    class Matrix
    {
        std::vector<std::vector<Type> > value;
        public:
            Matrix(int m, int n, Type d);
            Matrix(std::vector<std::vector<Type> > value);
            Matrix(Type *array, int m, int n);
            Matrix(const Matrix& other_matrix);
            ~Matrix();
            void show();
            std::vector<std::vector<Type> > get_value() const;

            Matrix<Type> transpose();
            Matrix<Type> inverse();

            Matrix<Type>& operator=(const Matrix<Type>& other_matrix);
            Matrix<Type> operator+(const Matrix<Type>& other_matrix) const;
            Matrix<Type> operator-(const Matrix<Type>& other_matrix) const;
            Matrix<Type> operator*(const Matrix<Type>& other_matrix) const;
            // Matrix<Type> operator[](int x);
            Type& operator()(int x, int y);
            Type& operator()(int x);
    };
#endif