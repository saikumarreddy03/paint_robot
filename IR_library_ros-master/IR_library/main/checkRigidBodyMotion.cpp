#include <cmath>
#include <iostream>
#include "TypeDefs.hpp"
#include "RigidBodyMotion.hpp"
#include "ForwardKinematics.hpp"

using namespace IRlibrary;
int main(int argc, char **argv)
{
	double theta = M_PI/3;
	SO3Mat rotMat;
	rotMat << cos(theta) , -sin(theta), 0,
				 sin(theta), cos(theta), 0,
				 0, 0, 1;
	std::cout << "Rotation Matrix " << std::endl;
	std::cout << rotMat << std::endl;
	std::cout << "Rotation matrix inverse " << std::endl;
	std::cout << RotInv(rotMat) << std::endl;
	std::cout << "Rotation matrix inverse using inbuilt function inverse() " << std::endl;
	std::cout << rotMat.inverse() << std::endl;
	std::cout << "Rotation matrix transpose using inbuilt method transpose() " << std::endl;
	std::cout << rotMat.transpose() << std::endl;
	SE3Mat T;
	T << 0,0,-1,250,
			0,-1,0,-150,
			-1,0,0,200,
			0,0,0,1;
	Vec2 v2;
	Vec3 v3;
	Vec4 v4;
	v2 << 0,1;
	v3 << 0,0,1;
	v4 << 0,0,0,1;
	Vec3 vec;
	vec << 1,2,3;
	std::cout << " Trasnformation Matrix" << std::endl;
	std::cout << T << std::endl;
	std::cout << "Checking if matrix is SE3 " << std::endl;
	std::cout << isSE3(T) << std::endl;
	std::cout << "Checking if matrix is se3  " << std::endl;
	std::cout << isse3(T) << std::endl;
	std::cout << " Checking if matrix is so3 " << std::endl;
	std::cout << isso3(rotMat) << std::endl;
	std::cout << " Checking if matrix is SO3 " << std::endl;
	std::cout << isSO3(rotMat) << std::endl;
	std::cout << " Checking unit vector" << std::endl;
	std::cout << isUnit(v2) << std::endl;
	std::cout << " Checking unit vector" << std::endl;
	std::cout << isUnit(v3) << std::endl;
	std::cout << " Checking unit vector" << std::endl;
	std::cout << isUnit(v4) << std::endl;
	std::cout << "Converting vector to so3 " << std::endl;
	std::cout << VecToso3(vec) << std::endl;
	std::cout << "Converting so3 to vector " << std::endl;
	std::cout << so3ToVec(VecToso3(vec)) << std::endl;
	std::cout << "Extracting the rotation axis, omega_hat, and the rotation amount, theta " << std::endl;
	auto axis = AxisAng3(v3);
	std::cout << axis.theta << std::endl;
	std::cout << axis.omega << std::endl;
	so3Mat in_mat;
	in_mat << 0,-0.5,0.866,
					0.5,0,0,
					-0.866,0,0;
	std::cout << "Computing the rotation matrix R " << std::endl;
	std::cout << MatrixExp3(in_mat) << std::endl;
	std::cout << "Computing the matrix logarithm " << std::endl;
	std::cout << MatrixLog3(rotMat) << std::endl;
	std::cout << "Computing the 4x4 transformation matrix " << std::endl;
	std::cout << RpToTrans(rotMat,vec) << std::endl;
	std::cout << "Inverse of transformation matrix " << std::endl;
	std::cout << TransInv(RpToTrans(rotMat,vec)) << std::endl;
	Twist V;
	V << 1,2,3,4,5,6;
	std::cout << "se(3) matrix corresponding to a 6-vector twist V " << std::endl;
	std::cout << VecTose3(V) << std::endl;
	std::cout << "Twist from se(3) matrix " << std::endl;
	std::cout << se3ToVec(VecTose3(V)) << std::endl;
	std::cout << "Computing 6x6 adjoint matrix from T " << std::endl;
	std::cout << Adjoint(T) << std::endl;
	std::cout << "Normalized screw axis representation " << std::endl;
	std::cout << ScrewToAxis(v3,vec,3) << std::endl;
	std::cout << "Computing the homogeneous transformation matrix T in SE(3) " << std::endl;
	std::cout << MatrixExp6(VecTose3(V)) << std::endl;
	std::cout << "Computes the matrix logarithm se3mat in se(3) " << std::endl;
	std::cout << MatrixLog6(RpToTrans(rotMat,vec)) << std::endl;
	return 0;
}
