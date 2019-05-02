#ifndef Jacobian_HPP
#define Jacobian_HPP

#include <vector>
#include "RigidBodyMotion.hpp"
#include "TypeDefs.hpp"

#include <cmath>
#include <math.h>

namespace IRlibrary
{
	
	/** Computes the body Jacobian**/
	JacobianMat JacobianBody (std::vector <ScrewAxis> Blist, std::vector <double> thetaList)
	{
		size_t n = Blist.size();
		JacobianMat Jb (6,n);
		Jb.col(0) = Blist[0];
		SE3Mat T = SE3Mat::Identity();
		Twist temp(Blist[0].size());
		for (int i = int(thetaList.size()) - 2; i >= 0; i--) {
			temp << Blist[i + 1] * thetaList[i + 1];
			T = T * MatrixExp6(VecTose3(-1 * temp));
			Jb.col(i) = Adjoint(T) * Blist[i];
		}
		return Jb;
	}
	
	/** Computes the space Jacobian**/
	JacobianMat JacobianSpace (std::vector <ScrewAxis> Slist, std::vector <double> thetaList)
	{
		size_t n = Slist.size();
		JacobianMat Js (6,n);
		Js.col(0) = Slist[0];
		SE3Mat T = SE3Mat::Identity();
		Twist temp(Slist[0].size());
		for (int i = 1; i < int(thetaList.size()); i++) {
			temp << Slist[i - 1] * thetaList[i - 1];
			T = T * MatrixExp6(VecTose3(temp));
			Js.col(i) = Adjoint(T) * Slist[i];
		}

		return Js;
	}

} /* IRlibrary */ 

#endif /* ifndef Jacobian_HPP */
