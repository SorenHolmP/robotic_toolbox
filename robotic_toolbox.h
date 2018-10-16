#include <iostream>

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace rw::math;


#define REVOLUTE 1
#define PRISMATIC 0

Eigen::Matrix<float, 3,3> make_rotation_matrix(double R, double P, double Y);
std::vector<Eigen::VectorXd> inverse_RPY(Eigen::Matrix<float, 3,3> rot);

//Exercise 3.4
Eigen::Matrix<float,4,4> Tz(double q, int type);
Eigen::Matrix<float,4,4> get_homogenous_transform(double RPY[], double displacement[]);
Eigen::Matrix<float,4,4> get_base_to_joint_transform(std::vector<Eigen::Matrix<float,4,4>> Trefs, double q[], int type[], int i);
Eigen::Matrix<float,4,4> get_base_to_TCP_transform(std::vector<Eigen::Matrix<float,4,4>> Trefs, Eigen::Matrix<float,4,4> T_TCP, double q[], int type[]);

//Exercise 4.1
Eigen::MatrixXf get_manipulator_jacobian(std::vector<Eigen::Matrix4f> Trefs, Eigen::Matrix4f T_TCP, double q[], int type[]);


