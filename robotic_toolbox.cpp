#include "robotic_toolbox.h"

Eigen::Matrix<float, 3,3> make_rotation_matrix(double R, double P, double Y)
{
   Eigen::Matrix<float, 3,3> Rz; Rz.setZero();
   Eigen::Matrix<float, 3,3> Ry; Ry.setZero();
   Eigen::Matrix<float, 3,3> Rx; Rx.setZero();
   Rz(0,0) = cos(R); Rz(0,1) = -sin(R); Rz(1,0) = sin(R); Rz(1,1) = cos(R); Rz(2,2) = 1;
   Ry(0,0) = cos(P); Ry(0,2) = sin(P); Ry(1,1) = 1; Ry(2,0) = -sin(P); Ry(2,2) = cos(P);
   Rx(0,0) = 1; Rx(1,1) = cos(Y); Rx(1,2) = -sin(Y); Rx(2,1) = sin(Y); Rx(2,2) = cos(Y);

   Eigen::Matrix<float, 3,3> rot;
   rot = Rz * Ry * Rx;

   return rot;
}

std::vector<Eigen::VectorXd> inverse_RPY(Eigen::Matrix<float, 3,3> rot)
{
   //WARNING: SOMETHING IS WRONG WITH THIS FUNCTION. DOES NOT PRODUCE CORRECT ZYX EULER ANGLES :-(
   //Or does it...?
  std::vector<Eigen::VectorXd> container;
  double theta_y1, theta_x1, theta_z1, theta_y2, theta_x2, theta_z2;

  if(! (rot(2,2) == 1 && rot(2,2) == -1) )
  {
      theta_y1 = asin(-rot(2,0));
      theta_y2 = Pi - theta_y1;

      theta_x1 = atan2(cos(theta_y1)*rot(2,1), cos(theta_y1)*rot(2,2));
      theta_z1 = atan2(cos(theta_y1)*rot(1,0), cos(theta_y1)*rot(0,0));

      theta_x2 = atan2(cos(theta_y2)*rot(2,1), cos(theta_y2)*rot(2,2));
      theta_z2 = atan2(cos(theta_y2)*rot(1,0), cos(theta_y2)*rot(0,0));
      std::cout << "Hello from 1st if sentence" << std::endl;
      Eigen::VectorXd set1(3), set2(3);
      set1 << theta_z1, theta_y1, theta_x1;
      set2 << theta_z2, theta_y2, theta_x2;
      container.push_back(set1); container.push_back(set2);
  }
  else
  {
      if(rot(2,2) == 1)
      {
          theta_y1 = Pi/2;
          theta_z1 = 0; //Can be set to anything according to: http://www.gregslabaugh.net/publications/euler.pdf
          theta_x1 = atan2(rot(0,1), rot(0,2)) - theta_z1;
      }
      else //rot(2,2) == -1
      {
          theta_y1 = -Pi/2;
          theta_z1 = 0; //Can be set to anything according to: http://www.gregslabaugh.net/publications/euler.pdf
          theta_x1 = atan2(-rot(0,1), -rot(0,2)) + theta_z1;
      }
      Eigen::VectorXd set1(3);
      set1 << theta_z1, theta_y1, theta_x1;
      container.push_back(set1);
  }
  return container;
}

//UR5 robot.

Eigen::Matrix<float,4,4> get_homogenous_transform(double RPY[], double displacement[])
{
    Eigen::Matrix<float,4,4> T; T.setZero();
    Eigen::Matrix<float,3,3> rot = make_rotation_matrix(RPY[0],RPY[1],RPY[2]);
    T.block<3,3>(0,0) = rot;
    Eigen::Vector3f disp = {displacement[0], displacement[1], displacement[2]}; //Displacement vector
    T.block<3,1>(0,3) = disp;
    T(3,3) = 1;

    return T;
}

Eigen::Matrix<float,4,4> Tz(double q, int type) //Transform due to joint movement
{
   Eigen::Matrix<float,4,4> Tz; Tz.setZero();
   if(type == REVOLUTE)
   {
       Tz.block<3,3>(0,0) = make_rotation_matrix(q,0,0); //Eq 3.18.
       Tz(3,3) = 1;
   }
   if(type == PRISMATIC)
   {
       Tz.block<3,3>(0,0) = Eigen::Matrix<float,3,3>::Identity(3,3);
       Eigen::Vector3f disp = {0,0,q};
       Tz.block<3,1>(0,3) = disp;
       Tz(3,3) = 1;
   }
   return Tz;
}


Eigen::Matrix<float,4,4> get_base_to_joint_transform(std::vector<Eigen::Matrix<float,4,4>> Trefs, double q[], int type[], int i)
{
   Eigen::Matrix<float,4,4> T_Bi; T_Bi.setZero(); //The transform that takes us from the base frame to the ith joint frame

   //Update the frames with joint configuration:
   Eigen::Matrix<float,4,4> Trefs_new[i];

   for(int k = 0; k < i; k++)
       Trefs_new[k] = Trefs[k] * Tz(q[k],type[k]); //Eq 3.17 p.30

   T_Bi = Trefs_new[0];

   for(int k = 1; k < i; k++)
       T_Bi = T_Bi * Trefs_new[k]; //Multiply all the new reference frames together to form T_Bi

   return T_Bi;
}


Eigen::Matrix<float,4,4> get_base_to_TCP_transform(std::vector<Eigen::Matrix<float,4,4>> Trefs, Eigen::Matrix<float,4,4> T_TCP, double q[], int type[])
{
   Eigen::Matrix<float,4,4> T_Bi; T_Bi.setZero(); //The transform that takes us from the base frame to the ith joint frame

   //Update the frames with joint configuration:
   int i = Trefs.size();
   Eigen::Matrix<float,4,4> Trefs_new[i];


   for(int k = 0; k < i; k++)
       Trefs_new[k] = Trefs[k] * Tz(q[k],type[k]); //Eq 3.17 p.30

   T_Bi = Trefs_new[0];

   for(int k = 1; k < i; k++)
       T_Bi = T_Bi * Trefs_new[k]; //Multiply all the new reference frames together to form T_Bi

   return T_Bi * T_TCP; //A bit of a hack...
}


Eigen::MatrixXf get_manipulator_jacobian(std::vector<Eigen::Matrix4f> Trefs, Eigen::Matrix4f T_TCP, double q[], int type[])
{
    int num_joints = Trefs.size();
    std::vector<Eigen::Matrix4f> updated_Trefs;
    std::vector<Eigen::Vector3f> p;
    std::vector<Eigen::Vector3f> z;

    for(int i = 1; i < Trefs.size() + 1; i++)
    {
        Eigen::Matrix4f Tref_updated = get_base_to_joint_transform(Trefs, q, type, i);
        p.push_back( Tref_updated.block<3,1>(0,3) );                     //Collect all position vectors wrt. base frame
        z.push_back( Tref_updated.block<3,1>(0,2) );                     //Collect all z orientations wrt. base frame
        updated_Trefs.push_back( Tref_updated );
    }
    Eigen::Matrix4f T_B_TCP = updated_Trefs[num_joints - 1] * T_TCP;     //Transform from base to the TCP is just last transform multiplied by
                                                                         //independent (wrt. joint variables)tool transform
    Eigen::Vector3f pbt = T_B_TCP.block<3,1>(0,3);                       //Vector from origin of base frame to TCP

    //Fill up jacobian:
    Eigen::MatrixXf J;
    J.resize(6, num_joints); //Compute size of jacobian at runtime.
    for(int i = 0; i < Trefs.size(); i++)
    {
        J.block<3,1>(0,i) = z[i].cross(pbt - p[i]);
        J.block<3,1>(3,i) = z[i];
    }
    return J;
}
