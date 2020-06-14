
#include "tensor.h"

// matrix3333
template<int dim>
Tensor<dim>::Tensor()
{

}

template<int dim>
Tensor<dim>::Tensor(const Tensor<dim>& other)
{
	for (unsigned int row = 0; row != dim; ++row)
	{
		for (unsigned int col = 0; col != dim; ++col)
		{
			mat[row][col] = other.mat[row][col];
		}
	}
}

template<int dim>
void Tensor<dim>::SetZero()
{
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			mat[row][col] = Mat::Zero();
		}
	}
}

template<int dim>
void Tensor<dim>::SetIdentity()
{
	for (unsigned int row = 0; row != 3; ++row)
	{
		for (unsigned int col = 0; col != 3; ++col)
		{
			mat[row][col] = Mat::Zero();
			mat[row][col](row, col) = 1.0;
		}
	}
}

template<int dim>
Eigen::Matrix<real, dim, dim, 0, dim, dim>& Tensor<dim>::operator() (int row, int col)
{
	assert(row >= 0 && row < dim && col >= 0 && col < dim);
	return mat[row][col];
}

template<int dim>
Tensor<dim> Tensor<dim>::operator+ (const Tensor<dim>& plus)
{
	Tensor<dim> res;
	for (unsigned int row = 0; row != dim; ++row)
	{
		for (unsigned int col = 0; col != dim; ++col)
		{
			res.mat[row][col] = mat[row][col] + plus.mat[row][col];
		}
	}
	return res;
}

template<int dim>
Tensor<dim> Tensor<dim>::operator- (const Tensor<dim>& minus)
{
	Tensor<dim> res;
	for (unsigned int row = 0; row != dim; ++row)
	{
		for (unsigned int col = 0; col != dim; ++col)
		{
			res.mat[row][col] = mat[row][col] - minus.mat[row][col];
		}
	}
	return res;
}

template<int dim>
Tensor<dim> Tensor<dim>::operator* (const Mat& multi)
{
	Tensor<dim> res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res.mat[i][j].setZero();
			for (unsigned int k = 0; k < dim; k++)
			{
				res.mat[i][j] += mat[i][k] * multi(k, j);
			}
		}
	}
	return res;
}

template<int dim>
Tensor<dim> operator* (const Eigen::Matrix<real, dim, dim, 0, dim, dim>& multi1, Tensor<dim>& multi2)
{
	Tensor<dim> res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res(i, j).setZero();
			for (unsigned int k = 0; k < dim; k++)
			{
				res(i, j) += multi1(i, k) * multi2(k, j);
			}
		}
	}
	return res;
}

template<int dim>
Tensor<dim> Tensor<dim>::operator* (real multi)
{
	Tensor res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res.mat[i][j] = mat[i][j] * multi;
		}
	}
	return res;
}

template<int dim>
Tensor<dim> operator* (real multi1, Tensor<dim> & multi2)
{
	Tensor<dim> res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res(i, j) = multi1 * multi2(i, j);
		}
	}
	return res;
}

template<int dim>
Tensor<dim> Tensor<dim> ::transpose()
{
	Tensor<dim> res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res(i, j) = mat[j][i];
		}
	}
	return res;
}

template<int dim>
Eigen::Matrix<real, dim, dim, 0, dim, dim> Tensor<dim>::Contract(const Mat& multi)
{
	Mat res;
	res.setZero();
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res += mat[i][j] * multi(i, j);
		}
	}
	return res;
}

template<int dim>
Tensor<dim> Tensor<dim>::Contract(Tensor<dim>& multi)
{
	Tensor<dim> res;
	for (unsigned int i = 0; i < dim; ++i)
	{
		for (unsigned int j = 0; j < dim; ++j)
		{
			res(i, j) = this->Contract(multi(i, j));
		}
	}
	return res;
}

template class Tensor<2>;
template class Tensor<3>;