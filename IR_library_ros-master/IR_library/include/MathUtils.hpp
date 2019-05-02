#ifndef MATHUTILS
#define MATHUTILS

#include <cmath>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double, double eps = 1e-10); 

	/** Wraps angle (in radians) between 0 to 2 pi **/
	double wrapTo2PI (double); 

	/** Wraps angle (in radians) between -pi to pi **/
	double wrapToPI (double); 

	/** Converts angle from degree to radians **/
	double deg2rad (double); 

	/** Converts angle from radians to degree **/
	double rad2deg (double); 


} /* IRlibrary */
#endif /* ifndef MATHUTILS */
