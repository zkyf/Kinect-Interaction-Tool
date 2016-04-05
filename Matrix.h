#define _USE_MATH_DEFINES

#include <math.h>

#ifndef _MATRIX_H_
#define _MATRIX_H_

namespace ljxMat
{
	class Matrix
	{
		private:
		unsigned int width;
		unsigned int height;
		double       *mat;
		//Error Code
		enum ErrorCode
		{
			Succeeded,
			SizeFail,
			Exceed,
			MatNull,
			Singular
		};
		ErrorCode result;

		public:
		//Constructors & Deconstructors
		Matrix(const unsigned int _height, const unsigned int _width);
		Matrix(const unsigned int _height, const unsigned int _width, const double *_mat);
		Matrix(Matrix &matrix);
		~Matrix();

		//Functions
		ErrorCode    SetIdentity();
		void         SetToZero();
		double&      at(unsigned int row, unsigned int col);
		unsigned int Width();
		unsigned int Height();

		//Operators
		void   operator=(Matrix& matrix);
		Matrix operator*(Matrix& matrix);
		Matrix operator*(double x);
		Matrix operator+(Matrix& matrix);
		Matrix operator-(Matrix& matrix);
		Matrix operator!(void); //×ªÖÃ
		Matrix operator~(void); //Äæ
	};
}

#endif