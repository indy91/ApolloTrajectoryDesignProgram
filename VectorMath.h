/***************************************************************************
This file is part of the Apollo Trajectory Design Program.

The Apollo Trajectory Design Program is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The Apollo Trajectory Design Program is distributed in the hope that it will
be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public Licensealong with
the Apollo Trajectory Design Program. If not, see <https://www.gnu.org/licenses/>.

**************************************************************************/

#pragma once

#include <cmath>

/**
 * \brief 3-element vector
 */
typedef union {
	double data[3];               ///< array data interface
	struct { double x, y, z; };   ///< named data interface
} VECTOR3;

/**
 * \brief 3x3-element matrix
 */
typedef union {
	double data[9];               ///< array data interface (row-sorted)
	struct { double m11, m12, m13, m21, m22, m23, m31, m32, m33; }; ///< named data interface
} MATRIX3;

/**
 * \ingroup vec
 * \brief Vector composition
 *
 * Returns a vector composed of the three provided arguments
 * \param x x-component
 * \param y y-component
 * \param z z-component
 * \return vector defined as (x,y,z)
 */
inline VECTOR3 _V(double x, double y, double z)
{
	VECTOR3 vec = { x,y,z }; return vec;
}

/**
 * \ingroup vec
 * \brief Vector addition
 * \param a first vector operand
 * \param b second vector operand
 * \return Result of a+b.
 */
inline VECTOR3 operator+ (const VECTOR3& a, const VECTOR3& b)
{
	VECTOR3 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}

/**
 * \ingroup vec
 * \brief Vector subtraction
 * \param a first vector operand
 * \param b second vector operand
 * \return Result of a-b.
 */
inline VECTOR3 operator- (const VECTOR3& a, const VECTOR3& b)
{
	VECTOR3 c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return c;
}

/**
 * \ingroup vec
 * \brief Multiplication of vector with scalar
 * \param a vector operand
 * \param f scalar operand
 * \return Result of element-wise a*f.
 */
inline VECTOR3 operator* (const VECTOR3& a, const double f)
{
	VECTOR3 c;
	c.x = a.x * f;
	c.y = a.y * f;
	c.z = a.z * f;
	return c;
}

/**
 * \ingroup vec
 * \brief Division of vector by a scalar
 * \param a vector operand
 * \param f scalar operand
 * \return Result of element-wise a/f.
 */
inline VECTOR3 operator/ (const VECTOR3& a, const double f)
{
	VECTOR3 c;
	c.x = a.x / f;
	c.y = a.y / f;
	c.z = a.z / f;
	return c;
}

/**
 * \ingroup vec
 * \brief Vector unary minus -a
 * \param[in] a Vector operand
 * \return Negative vector (-a.x, -a.y, -a.z)
 */
inline VECTOR3 operator- (const VECTOR3& a)
{
	VECTOR3 c;
	c.x = -a.x;
	c.y = -a.y;
	c.z = -a.z;
	return c;
}

/**
 * \ingroup vec
 * \brief Vector addition-assignment a += b
 * \param[in,out] a Left-hand vector operand
 * \param[in] b Right-hand vector operand
 * \return Replaces a with a+b and returns the result.
 */
inline VECTOR3& operator+= (VECTOR3& a, const VECTOR3& b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

/**
 * \ingroup vec
 * \brief Length (L2-norm) of a vector
 * \param a Vector operand
 * \return Vector norm |<b>a</b>|<sub>2</sub>
 */
inline double length(const VECTOR3& a)
{
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

/**
 * \ingroup vec
 * \brief Returns normalised vector
 *
 * Returns a vector of length 1 with the same direction
 * as the argument vector.
 * \param[in] a Vector argument
 * \return Normalised vector.
 * \note The length of a must be greater than 0.
 */
inline VECTOR3 unit(const VECTOR3& a)
{
	return a / length(a);
}

/**
 * \ingroup vec
 * \brief Scalar (inner, dot) product of two vectors
 * \param[in] a First vector operand
 * \param[in] b Second vector operand
 * \return Scalar product <b>ab</b>
 */
inline double dotp(const VECTOR3& a, const VECTOR3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * \ingroup vec
 * \brief Vector (cross) product of two vectors
 * \param[in] a First vector operand
 * \param[in] b Second vector operand
 * \return Vector product <b>a</b>x<b>b</b>
 */
inline VECTOR3 crossp(const VECTOR3& a, const VECTOR3& b)
{
	return _V(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y);
}

/**
 * \ingroup vec
 * \brief Matrix composition
 *
 * Returns a matrix composed of the provided elements.
 * \return
 * \f$
 *  \left(\begin{array}{ccc}
 *  m_{11} & m_{12} & m_{13} \\
 *  m_{21} & m_{22} & m_{23} \\
 *  m_{31} & m_{32} & m_{33}
 *  \end{array}\right)
 * \f$
 */
inline MATRIX3 _M(double m11, double m12, double m13,
	double m21, double m22, double m23,
	double m31, double m32, double m33)
{
	MATRIX3 mat = { m11,m12,m13,  m21,m22,m23,  m31,m32,m33 };
	return mat;
}

/**
 * \ingroup vec
 * \brief Matrix-vector multiplication
 * \param[in] A matrix operand
 * \param[in] b vector operand
 * \return Result of <b>Ab</b>
 */
inline VECTOR3 mul(const MATRIX3& A, const VECTOR3& b)
{
	return _V(
		A.m11 * b.x + A.m12 * b.y + A.m13 * b.z,
		A.m21 * b.x + A.m22 * b.y + A.m23 * b.z,
		A.m31 * b.x + A.m32 * b.y + A.m33 * b.z);
}

/**
 * \ingroup vec
 * \brief Matrix transpose-vector multiplication
 * \param[in] A matrix operand
 * \param[in] b vector operand
 * \return Result of <b>A</b><sup>T</sup><b>b</b>
 */
inline VECTOR3 tmul(const MATRIX3& A, const VECTOR3& b)
{
	return _V(
		A.m11 * b.x + A.m21 * b.y + A.m31 * b.z,
		A.m12 * b.x + A.m22 * b.y + A.m32 * b.z,
		A.m13 * b.x + A.m23 * b.y + A.m33 * b.z);
}

/**
 * \ingroup vec
 * \brief Matrix-matrix multiplication
 * \param[in] A First matrix operand
 * \param[in] B Second matrix operand
 * \return Result of <b>AB</b>
 */
inline MATRIX3 mul(const MATRIX3& A, const MATRIX3& B)
{
	MATRIX3 mat = {
		A.m11 * B.m11 + A.m12 * B.m21 + A.m13 * B.m31, A.m11 * B.m12 + A.m12 * B.m22 + A.m13 * B.m32, A.m11 * B.m13 + A.m12 * B.m23 + A.m13 * B.m33,
		A.m21 * B.m11 + A.m22 * B.m21 + A.m23 * B.m31, A.m21 * B.m12 + A.m22 * B.m22 + A.m23 * B.m32, A.m21 * B.m13 + A.m22 * B.m23 + A.m23 * B.m33,
		A.m31 * B.m11 + A.m32 * B.m21 + A.m33 * B.m31, A.m31 * B.m12 + A.m32 * B.m22 + A.m33 * B.m32, A.m31 * B.m13 + A.m32 * B.m23 + A.m33 * B.m33
	};
	return mat;
}