#ifndef INVERSEKINEMATICS_HPP
#define INVERSEKINEMATICS_HPP

#include <vector>
#include "TypeDefs.hpp"
#include <eigen3/Eigen/QR>
#include <Jacobian.hpp>
#include <RigidBodyMotion.hpp>
#include <ForwardKinematics.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigenvalues>

namespace IRlibrary
{
    bool IKinBody(std::vector <ScrewAxis> const &, SE3Mat const &, SE3Mat const &, Eigen::VectorXd const &,Eigen::VectorXd &, double eps_omg = 1e-10, double eps_v = 1e-10, size_t maxItr = 100);
    
    bool IKinSpace(std::vector <ScrewAxis> const &, SE3Mat const &, SE3Mat const &, Eigen::VectorXd const &,Eigen::VectorXd &, double eps_omg = 1e-10, double eps_v = 1e-10, size_t maxItr = 100);

} /* IRlibrary */ 

#endif /* ifndef INVERSEKINEMATICS_HPP */
