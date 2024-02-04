#include "eigen3/Eigen/Dense"
#include "iiwa7_ik/inverse_kinematics.hpp"
#include "pinocchio/math/rpy.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/workspace/models/iiwa7"
#endif

int main()
{
    // Example usage
    pinocchio::SE3 des_pose =
        pinocchio::SE3(pinocchio::rpy::rpyToMatrix(0.2, 0.0, 0.0), Eigen::Vector3d(0.3, 0.3, 1.0));
    double nsparam = 0.0;
    uint8_t rconf = 0b000;
    Eigen::VectorXd q(7);

    InverseKinematics ik = InverseKinematics();

    ik.compute(des_pose, nsparam, rconf, q);

    // Display joint angles, shoulder matrices, and wrist matrices
    std::cout << "Joint Angles: " << q.transpose() << std::endl;

    std::cout << "desired End-effector pose: \n" << des_pose << std::endl;

    pinocchio::SE3 actual_pose;

    std::cout << "actual End-effector pose: \n" << ik.forward_kinematics(q, actual_pose) << std::endl;

    return 0;
}
