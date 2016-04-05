#include "Matrix.h"
#include <stdio.h>
#include <stdlib.h>

using namespace ljxMat;

Matrix::Matrix(const unsigned int _height, const unsigned int _width) :
width(_width), height(_height), mat(NULL)
{
	mat = new double[width*height];
	SetToZero();
}

Matrix::Matrix(const unsigned int _height, const unsigned int _width, const double *_mat) :
width(_width), height(_height), mat(NULL)
{
	mat = new double[width*height];
	SetToZero();
	if (!_mat)
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				at(i, j) = _mat[i*width + j];
			}
		}
	}
}

Matrix::Matrix(Matrix &matrix)
{
	width = matrix.width;
	height = matrix.height;
	mat = new double[width*height];
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			mat[i*width + j] = matrix.mat[i*width + j];
		}
	}
}

Matrix::~Matrix(){ delete[] mat; }

Matrix::ErrorCode Matrix::SetIdentity()
{
	if (width != height)
	{
		return SizeFail;
	}
	if (!mat)
	{
		return MatNull;
	}
	SetToZero();
	for (int i = 0; i < height; i++)
	{
		mat[i*width + i] = 1;
	}
	return Succeeded;
}

void Matrix::SetToZero()
{
	if (!mat) return;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			mat[i*width + j] = 0;
		}
	}
}

double& Matrix::at(unsigned int row, unsigned int col) { return mat[row*width + col]; }
unsigned int Matrix::Width(){ return width; }
unsigned int Matrix::Height() { return height; }

void Matrix::operator=(Matrix& matrix)
{
	width = matrix.width;
	height = matrix.height;
	if (mat)
	{
		delete[] mat;
	}
	mat = new double[width*height];
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			mat[i*width + j] = matrix.mat[i*width + j];
		}
	}
}

Matrix Matrix::operator*(Matrix& matrix)
{
	Matrix r(height, matrix.width);
	if (matrix.height != width)
	{
		r.result = SizeFail;
		return r;
	}
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < matrix.width; j++)
		{
			for (int k = 0; k < width; k++)
			{
				r.at(i, j) = this->at(i, k) * matrix.at(k, j);
			}
		}
	}
	return r;
}

Matrix Matrix::operator*(double x)
{
	Matrix r(height, width);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			r.at(i, j) = this->at(i, j)*x;
		}
	}
	return r;
}

Matrix Matrix::operator+(Matrix& matrix)
{
	Matrix r(height, width);
	if (matrix.width != width || matrix.height != height)
	{
		r.result = SizeFail;
		return r;
	}
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			r.at(i, j) = at(i, j) + matrix.at(i, j);
		}
	}
	return r;
}

Matrix Matrix::operator-(Matrix& matrix)
{
	Matrix r(height, width);
	if (matrix.width != width || matrix.height != height)
	{
		r.result = SizeFail;
		return r;
	}
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			r.at(i, j) = at(i, j) - matrix.at(i, j);
		}
	}
	return r;
}

Matrix Matrix::operator!(void)
{
	Matrix r(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			r.at(j, i) = at(i, j);
		}
	}
	return r;
}

Matrix Matrix::operator~(void)
{
	Matrix r(height, width);
	Matrix copy(*this);
	if (width != height)
	{
		r.result = SizeFail;
		return r;
	}
	r.SetIdentity();
	for (int i = 0; i < height; i++)
	{
		if (copy.at(i, i) == 0)
		{
			int tochange = -1;
			for (int j = i + 1; j < height; j++)
			{
				if (copy.at(j, i) != 0)
				{
					tochange = j;
					break;
				}
			}
			if (tochange == -1)
			{
				r.result = Singular;
				return r;
			}
			for (int j = 0; j < width; j++)
			{
				double temp;
				temp = copy.at(tochange, j);
				copy.at(tochange, j) = copy.at(i, j);
				copy.at(i, j) = copy.at(tochange, j);
				temp = r.at(tochange, j);
				r.at(tochange, j) = r.at(i, j);
				r.at(i, j) = r.at(tochange, j);
			}
		}
		double ratio = copy.at(i, i);
		for (int j = 0; j < width; j++)
		{
			copy.at(i, j) /= ratio;
			r.at(i, j) /= ratio;
		}
		for (int j = 0; j < height && j != i; j++)
		{
			double ratio = copy.at(j, j);
			for (int k = 0; k < width; k++)
			{
				copy.at(j, k) -= copy.at(i, k*ratio);
				r.at(j, k) -= copy.at(i, k*ratio);
			}
		}
	}
	return r;
}