#ifndef Jacobian
#define Jacobian

#include <cmath>

namespace IRlibrary {

	/** Computes the body Jacobian**/
	JacobianMat JacobianBody (std::vector <ScrewAxis>, std::vector <double>); 
	
	/** Computes the space Jacobian**/
	JacobianMat JacobianSpace (std::vector <ScrewAxis>, std::vector <double>);


} /* IRlibrary */
#endif /* ifndef Jacobian */
