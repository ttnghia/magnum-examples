
#ifndef TENSOR_H_
#define TENSOR_H_

#include <Common/Setup.h>

template<int dim>
class Tensor // 3x3 matrix: each element is a 3x3 matrix
{
    using Mat = Eigen::Matrix<real, dim, dim, 0, dim, dim>;

public:
    Tensor();
    Tensor(const Tensor& other);
    ~Tensor() {}

    void SetZero();     // [0 0 0; 0 0 0; 0 0 0]; 0 = 3x3 zeros
    void SetIdentity(); //[I 0 0; 0 I 0; 0 0 I]; 0 = 3x3 zeros, I = 3x3 identity

    // operators
    Mat&          operator()(int row, int col);
    Tensor        operator+(const Tensor& plus);
    Tensor        operator-(const Tensor& minus);
    Tensor        operator*(const Mat& multi);
    friend Tensor operator*(const Mat& multi1, Tensor& multi2);
    Tensor        operator*(real multi);
    friend Tensor operator*(real multi1, Tensor& multi2);
    Tensor        transpose();
    Mat           Contract(const Mat& multi); // this operator is commutative
    Tensor        Contract(Tensor& multi);

protected:

    Mat mat[dim][dim];
};

#endif
