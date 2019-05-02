#include <RigidBodyMotion.hpp>

namespace IRlibrary {

	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat mat){
if ((mat.transpose() * mat).isIdentity()&& mat.determinant() == 1)
			return true;
		else
			return false;
	}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
		if (mat.transpose() == -mat)
			return true;
		else
			return false;
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat mat){

		SO3Mat R;
		R << mat(0,0), mat(0,1), mat(0,2),
				mat(1,0), mat(1,1), mat(1,2),
				mat(2,0), mat(2,1), mat(2,2);

		if (isSO3(R) && mat(3,0) == 0 && mat(3,1) == 0 && mat(3,2) == 0 && mat(3,3) == 1)
			return true;
		else
			return false;
	}

	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){

		SO3Mat R;
		R << mat(0,0), mat(0,1), mat(0,2),
				mat(1,0), mat(1,1), mat(1,2),
				mat(2,0), mat(2,1), mat(2,2);
		if (isso3(R) && mat(3,0) == 0 && mat(3,1) == 0 && mat(3,2) == 0 && mat(3,3) == 0)
			return true;
		else
			return false;
	}

	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
		if (vec.norm() == 1)
			return true;
		else
			return false;
	}
	bool isUnit(Vec3 vec){
		if (vec.norm() == 1)
			return true;
		else
			return false;
	}
	bool isUnit(Vec4 vec){
		if (vec.norm() == 1)
			return true;
		else
			return false;
	}

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
		return mat.transpose();
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
		so3Mat mat;
		mat << 0, -omega(2), omega(1),
				omega(2), 0, -omega(0), 
				-omega(1), omega(0), 0;
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		omega << -mat(1,2),
						-mat(2,0),
						-mat(0,1);
		return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3 expc3) {
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();
		axisAngle.omega = expc3.normalized();
		return axisAngle;
	}

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat in_mat){
		SO3Mat identityMat;
		
		AxisAngle axisAngle = AxisAng3(so3ToVec(in_mat));
		so3Mat skew_omega = VecToso3(axisAngle.omega);
		so3Mat skew_omega_square = skew_omega*skew_omega;

		SO3Mat R = Eigen::Matrix3d :: Identity () + (sin(axisAngle.theta)* skew_omega);
		R = R + (1-cos(axisAngle.theta)) * (skew_omega_square);

		return R;
	}

	/** Computes the matrix logaritomega_hathm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat in_mat){
		double acos_val = (in_mat.trace() - 1) / 2.0;
		Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(3, 3);
		if (acos_val >= 1)
			return mat;
		else if (acos_val <= -1) {
			Eigen::Vector3d omega;
			if (!nearZero(1 + in_mat(2, 2)))
				omega = (1.0 / std::sqrt(2 * (1 + in_mat(2, 2))))*Eigen::Vector3d(in_mat(0, 2), in_mat(1, 2), 1 + in_mat(2, 2));
			else if (!nearZero(1 + in_mat(1, 1)))
				omega = (1.0 / std::sqrt(2 * (1 + in_mat(1, 1))))*Eigen::Vector3d(in_mat(0, 1), 1 + in_mat(1, 1), in_mat(2, 1));
			else
				omega = (1.0 / std::sqrt(2 * (1 + in_mat(0, 0))))*Eigen::Vector3d(1 + in_mat(0, 0), in_mat(1, 0), in_mat(2, 0));
			mat = VecToso3(M_PI * omega);
			return mat;
		}
		else {
			double theta = std::acos(acos_val);
			mat = theta / 2.0 / sin(theta)*(in_mat - in_mat.transpose());
			return mat;
		}
		
	}

	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		T << R(0,0), R(0,1), R(0,2), p(0),
			R(1,0), R(1,1), R(1,2), p(1),
			R(2,0), R(2,1), R(2,2), p(2),
			0, 0, 0, 1;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		//R = mat.block<3, 3>(0, 0);
		R = mat.block<3, 3>(0, 0);
		p << mat(0,3), mat(1,3), mat(2,3);
	}

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat mat){

		SO3Mat R = mat.block<3, 3>(0, 0);
		Vec3 p;
		p << mat(0,3), mat(1,3), mat(2,3);
		SO3Mat R_transpose = R.transpose();
		Vec3 y = - R_transpose * p;
		mat << R_transpose(0,0), R_transpose(0,1), R_transpose(0,2), y(0),
					R_transpose(1,0), R_transpose(1,1), R_transpose(1,2), y(1),
					R_transpose(2,0), R_transpose(2,1), R_transpose(2,2), y(2),
					0, 0, 0, 1;
		return mat;
	}

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist V){
		se3Mat mat;
		Vec3 omega;
		omega << V(0),
						V(1),
						V(2);
		auto omega_s03 = VecToso3(omega);
		mat << omega_s03(0,0), omega_s03(0,1), omega_s03(0,2), V(3),
					omega_s03(1,0), omega_s03(1,1), omega_s03(1,2), V(4),
					omega_s03(2,0), omega_s03(2,1), omega_s03(2,2), V(5),
					0, 0, 0, 0;
		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat mat){
		Twist V;
		Vec3 omega;
		omega = so3ToVec(mat.block<3, 3>(0, 0));
		V << omega(0),
				omega(1),
				omega(2),
				mat(0,3),
				mat(1,3),
				mat(2,3);
		//V << 0, 0, 0, 0, 0, 0;
		return V;
	}

	/** Compute 6x6 adjoint matrix from T **/
	AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
		auto R = mat.block<3,3>(0,0);
		Vec3 p;
		p << mat(0,3), mat(1,3), mat(2,3);
		auto p_so3 = VecToso3(p);
		auto mat_product = p_so3 * R;
		adj_mat << R(0,0), R(0,1), R(0,2), 0, 0, 0,
						R(1,0), R(1,1), R(1,2), 0, 0, 0,
						R(2,0), R(2,1), R(2,2), 0, 0, 0,
						mat_product(0,0), mat_product(0,1), mat_product(0,2), R(0,0), R(0,1), R(0,2),
						mat_product(1,0), mat_product(1,1), mat_product(1,2), R(1,0), R(1,1), R(1,2),
						mat_product(2,0), mat_product(2,1), mat_product(2,2), R(2,0), R(2,1), R(2,2);
		return adj_mat;
	}

	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h) {
		ScrewAxis S;
		Vec3 v = q.cross(s) + (h * s);
		S << s(0),
				s(1),
				s(2),
				v(0),
				v(1),
				v(2);
		return S;
	}

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
		//S << 0, 0, 0, 0, 0, 1;
		Vec3 omega = STheta.block<3,1>(0,0);
		Vec3 vel = STheta.block<3,1>(3,0);
		theta = omega.norm();
        if (nearZero(theta))
        {
            if(nearZero(vel.norm()))
                theta = 1;
            else
                 theta = vel.norm(); 
        }             
        S << STheta / theta;
	}

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat in_mat){
		auto in_mat_block = in_mat.block<3, 3>(0, 0);
		auto omg_theta = so3ToVec(in_mat_block);
		SE3Mat mat;

		if (nearZero(omg_theta.norm())) {

			in_mat_block = Eigen::Matrix3d :: Identity ();
			omg_theta << in_mat(0, 3), in_mat(1, 3), in_mat(2, 3);
			mat << in_mat_block, omg_theta,
				0, 0, 0, 1;
			return mat;
		}

		else {

			auto angle = AxisAng3(omg_theta);

			auto omg_mat = in_mat.block<3, 3>(0, 0) / angle.theta;
			so3Mat exp = Eigen::Matrix3d :: Identity () * angle.theta + (1 - cos(angle.theta)) * omg_mat + ((angle.theta - sin(angle.theta)) * (omg_mat * omg_mat));

			Vec3 linear(in_mat(0, 3), in_mat(1, 3), in_mat(2, 3));
			auto theta_v = (exp*linear) / angle.theta;

			mat << MatrixExp3(in_mat_block), theta_v,
				0, 0, 0, 1;
			return mat;
               }
	}

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat;
		SO3Mat R;
		Vec3 p;
		TransToRp(T,R,p);
		so3Mat omega = MatrixLog3(R);
		if(omega.isApprox(Eigen::Matrix3d::Zero()))
		{
			mat << Eigen::Matrix3d::Zero(), p.normalized(),
							0,0,0,0;
		}
		else
		{
			double theta = std::acos((R.trace() - 1) / 2.0);
			Vec3 vel = (Eigen::Matrix3d::Identity() - omega/2 + ((1/theta) - ((1.0/tan(theta / 2))/2))*(omega*omega)/theta) * p;
			mat << omega, vel,
							0,0,0,0;
		}
		
		return mat;
	}
	

} /* IRlibrary */ 


