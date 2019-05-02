#ifndef MATHUTILS
#define MATHUTILS
#include <math.h>
//#include <cmath>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps = 1e-10) {
		return std::abs(val) <= eps;
	}

	/** Wraps angle between 0 to 2 pi **/
	double wrapTo2PI (double val) {
		val = fmod(val,2*M_PI);
		if (val < 0)
			val += 2 * M_PI;
		return val;
	}

	/** Wraps angle between -pi to pi **/
	double wrapToPI (double val) {
		val = fmod((val + M_PI) ,  2*M_PI);
		if (val < 0)
			val += 2*M_PI;
		val = val - M_PI;
		return val;
	}

} /* IRlibrary */
#endif /* ifndef MATHUTILS */