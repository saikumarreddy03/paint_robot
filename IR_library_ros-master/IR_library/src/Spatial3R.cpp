#include "Spatial3R.hpp"

namespace IRlibrary
{
	void Spatial3R::zeroForwardKinematics() {
		x = base;
		axisAngle.omega = {0, 0, 1};
		axisAngle.theta = 0.0;
	}
	bool  Spatial3R::inverseKinematics(Vec3 const & x_in) {
		auto xyp = x_in - base;
		//double l1 = l[0];
		//double l2 = l[1];
		//double l1 = l[2];
		double pSqr = xyp.squaredNorm();
		/*
		if (pSqr > (l1+l2) *(l1+l2) || pSqr < (l1-l2) *(l1-l2)){
			std::cout << "Planar2R::inverseKinematics [Warning] point outside workspace\n";
			return 1;
		}
		if (pSqr > 0.99 *(l1+l2) *(l1+l2) || pSqr < 1.01 * (l1-l2) *(l1-l2)){
			std::cout << "Planar2R::inverseKinematics [Warning] point close to singularity\n";
			return 1;
		}
		*/
		x  = x_in;
		double theta1 = atan2(xyp[1],xyp[0]);
		double s = xyp[2] - l[0];
		double r = std::sqrt(xyp[0]*xyp[0] + xyp[1]*xyp[1]);
		double theta2 = atan2(s,r);
		double theta3= -l[1] + std::sqrt( xyp[0]*xyp[0] + xyp[1]*xyp[1]+( (xyp[2] - l[0]) * (xyp[2] - l[0]) ) );
		q[0] = theta1;
		q[1] = theta2;
		q[2] = theta3;
	return 0;
	}
} /* IRlibrary */ 


