#ifndef RIGIDBODYMOTION_H
#define RIGIDBODYMOTION_H

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <TypeDefs.hpp>
#include <MathUtils.hpp>

namespace IRlibrary {


	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat);

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat);

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat);

	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat);

	/** Checks if the vector is unit **/
	bool isUnit(Vec2);
	bool isUnit(Vec3);
	bool isUnit(Vec4);

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat);

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3);

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat);

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3);

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat);

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat);

	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat, Vec3);

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat, SO3Mat &, Vec3 &);

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat);

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist);

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat);

	/** Compute 6x6 adjoint matrix from T **/
	AdjMat Adjoint(SE3Mat);

	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3, Vec3, double);

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist, ScrewAxis &, double &);

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat);

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat);

} /* IRlibrary */ 

#endif /* ifndef RIGIDBODYMOTION_H */
