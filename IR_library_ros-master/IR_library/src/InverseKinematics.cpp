#include <InverseKinematics.hpp>

namespace IRlibrary
{

    bool IKinBody(std::vector <ScrewAxis> const &Blist, SE3Mat const &M, SE3Mat const &T,Eigen::VectorXd const &init_thetalist,Eigen::VectorXd &thetalist,double const eps_omg, double const  eps_v, size_t maxItr) 
    {
		thetalist = init_thetalist;
        SE3Mat Tsb = SE3Mat::Identity();
        Twist Vs = Twist::Zero();
       for(size_t i = 0; i < maxItr; i++)
        {
            std::vector <double> thetaVec (thetalist.data(),thetalist.data() + thetalist.size());
            Tsb = FKinBody(M, Blist, thetaVec);
            Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb)*T));
            if(Vs.head(3).norm() < eps_omg and Vs.tail(3).norm() < eps_v)
                return true;
            JacobianMat Jb = JacobianBody(Blist, thetaVec);
            auto JbCOD = Jb.completeOrthogonalDecomposition();
            if((unsigned)JbCOD.rank() < Blist.size())
            {
                std::cout << "Jacobian not invertible";
                return 1;
            }
            Eigen::MatrixXd pinv = JbCOD.pseudoInverse();
            thetalist += pinv * Vs;
            }
        return 0; 
     }

    bool IKinSpace(std::vector <ScrewAxis> const &Slist, SE3Mat const &M, SE3Mat const &T,Eigen::VectorXd const &init_thetalist,Eigen::VectorXd &thetalist, double const eps_omg, double const eps_v, size_t maxItr) {
		thetalist = init_thetalist;
        SE3Mat Tsb = SE3Mat::Identity();
        Twist Vs = Twist::Zero();
        for(size_t i = 0; i < maxItr; i++)
        {
        std::vector <double> thetaVec (thetalist.data(),thetalist.data() + thetalist.size());
        Tsb = FKinSpace(M, Slist, thetaVec);
        Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb)*T));
        if(Vs.head(3).norm() < eps_omg and Vs.tail(3).norm() < eps_v)
            return true;
        JacobianMat Js = JacobianBody(Slist, thetaVec);
        auto JsCOD = Js.completeOrthogonalDecomposition();
        if((unsigned)JsCOD.rank() < Slist.size())
        {
            std::cout << "Jacobian not invertible";
            return 1;
        }
        Eigen::MatrixXd pinv = JsCOD.pseudoInverse();
        thetalist += pinv * Vs;
        }
        return 0;
     } 

} /* IRlibrary */
 


