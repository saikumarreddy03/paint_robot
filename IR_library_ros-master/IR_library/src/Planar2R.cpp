#include "Planar2R.hpp"

namespace IRlibrary
{
	void Planar2R::zeroForwardKinematics() {
		xy[0] = base[0] + l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]);
		xy[1] = base[1] + l[0] *  sin(q[0]) +  l[1] * sin(q[0] + q[1]);
	}
	bool  Planar2R::inverseKinematics(Vec2 const & xy_in) {
		auto xyp = xy_in - base;
		double l1 = l[0];
		double l2 = l[1];
		double pSqr = xyp.squaredNorm();
		if (pSqr > (l1+l2) *(l1+l2) || pSqr < (l1-l2) *(l1-l2)){
			std::cout << "Planar2R::inverseKinematics [Warning] point outside workspace\n";
			return 1;
		}
		if (pSqr > 0.99 *(l1+l2) *(l1+l2) || pSqr < 1.01 * (l1-l2) *(l1-l2)){
			std::cout << "Planar2R::inverseKinematics [Warning] point close to singularity\n";
			return 1;
		}
		xy = xy_in;
		double beta = acos((l1*l1 + l2*l2 -pSqr) / (2*l1*l2));
		double alpha= acos((pSqr + l1*l1 - l2*l2)/(2*l1*std::sqrt(pSqr)));
		double  gamma = atan2(xyp[1],xyp[0]);
		if (righty) {
			q[0] = gamma - alpha;
			q[1] = M_PI - beta;
		}
		else {
			q[0] = gamma + alpha;
			q[1] = -M_PI + beta;
		}
	return 0;
	}
} 
/* IRlibrary */


