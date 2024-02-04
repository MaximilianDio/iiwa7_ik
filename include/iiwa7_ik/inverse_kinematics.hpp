#ifndef MARS_MODEL_INVERSE_KINEMATICS_HPP
#define MARS_MODEL_INVERSE_KINEMATICS_HPP

#include "eigen3/Eigen/Dense"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/skew.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

constexpr int N_JOINTS = 7;
// TODO: currently only iiwa7 is used. When other robots are required, structure should be changed.
constexpr double l_iiwa7[] = {0.34, 0.4, 0.4, 0.126};

using Interval = std::pair<double, double>;

Interval tan_joint_limit(double an, double ad, double bn, double bd, double cn, double cd, const Interval &joint_angle)
{
    // TODO implement
    return Interval(0, 0);
}

Interval cos_joint_limit(double a, double b, double c, double sign, const Interval &joint_angle)
{
    // TODO implement
    return Interval(0, 0);
}

Interval IntersectIntervals(const std::vector<Interval> &intervals)
{
    // TODO implement
    return Interval(0, 0);
}


class InverseKinematics
{
public:
    InverseKinematics()
    {
        // fill with -pi and pi (joint 4 does not affect psi!)
        psi_limits = std::vector<Interval>(N_JOINTS - 1, Interval(-M_PI, M_PI));
    }

    /**
     * @brief Compute the inverse kinematics for the iiwa7
     * 
     * @param pose desired end-effector pose
     * @param nsparam null space parameter
     * @param rconf robot configuration
     * @param joints output joint angles
    */
    void compute(const pinocchio::SE3 &pose, double nsparam, uint8_t rconf, Eigen::VectorXd &joints)
    {
        // use bitshift to extract the robot configuration
        const double shoulder = -(rconf >> 2) * 2 + 1;    // [1,x,x]->1, [0,x,x]->-1
        const double elbow = -((rconf >> 1) & 1) * 2 + 1; // [x,1,x]->1, [x,0,x]->-1
        const double wrist = -(rconf & 1) * 2 + 1;        // [x,x,1]->1, [x,x,0]->-1

        // computational tolerance
        const double tol = 1e-8;

        // compute positions of shoulder, wrist, and end-effector
        xend = pose.translation();
        xw0 = xend - pose.rotation() * xwt;
        xsw = xw0 - xs0;
        usw = xsw.normalized();

        // ensure that the desired end-effector pose is within the workspace
        if ((xsw.norm() < lbs + lse + lew) && (xsw.norm() > lbs - lse - lew))
            std::runtime_error("Desired pose is outside reachable workspace.");

        if (std::abs((xsw.norm() * xsw.norm() - lse * lse - lew * lew) - (2 * lse * lew)) > tol)
            std::runtime_error("Elbow singularity! Tip at reach limit.");

        // Joint 4 can be computed directly (independent of other joints)
        joints(3) = elbow * std::acos((xsw.norm() * xsw.norm() - lse * lse - lew * lew) / (2 * lse * lew));

        T34 = dh_calc(dh(3, 0), dh(3, 1), dh(3, 2), joints(3));

        // compute T03_o
        reference_plane(elbow, joints);

        // Shoulder Joints
        skew_usw = pinocchio::skew(usw);
        As = skew_usw * T03_o.rotation();
        Bs = -skew_usw * skew_usw * T03_o.rotation();
        Cs = usw * usw.transpose() * T03_o.rotation();
        // Wrist Joints
        Aw = T34.rotation().transpose() * As.transpose() * pose.rotation();
        Bw = T34.rotation().transpose() * Bs.transpose() * pose.rotation();
        Cw = T34.rotation().transpose() * Cs.transpose() * pose.rotation();

        // [ ] 1) TODO: compute psi limits for each joint
        // [ ] 2) TODO: compute psi given the previous psi

        // up to here the null space parameter is not used
        double psi = nsparam;
        R03 = As * std::sin(psi) + Bs * std::cos(psi) + Cs;
        R47 = Aw * std::sin(psi) + Bw * std::cos(psi) + Cw;

        joints(0) = std::atan2(shoulder * R03(1, 1), shoulder * R03(0, 1));
        joints(1) = shoulder * std::acos(R03(2, 1));
        joints(2) = std::atan2(-shoulder * R03(2, 2), -shoulder * R03(2, 0));

        joints(4) = std::atan2(wrist * R47(1, 2), wrist * R47(0, 2));
        joints(5) = wrist * std::acos(R47(2, 2));
        joints(6) = std::atan2(wrist * R47(2, 1), -wrist * R47(2, 0));
    }

    pinocchio::SE3 &forward_kinematics(const Eigen::VectorXd &joints, pinocchio::SE3 &T07)
    {
        // Lower arm transformation
        // clang-format off
        T07 =
            dh_calc(dh(0, 0), dh(0, 1), dh(0, 2), joints(0)) * 
            dh_calc(dh(1, 0), dh(1, 1), dh(1, 2), joints(1)) *
            dh_calc(dh(2, 0), dh(2, 1), dh(2, 2), joints(2)) * 
            dh_calc(dh(3, 0), dh(3, 1), dh(3, 2), joints(3)) *
            dh_calc(dh(4, 0), dh(4, 1), dh(4, 2), joints(4)) * 
            dh_calc(dh(5, 0), dh(5, 1), dh(5, 2), joints(5)) *
            dh_calc(dh(6, 0), dh(6, 1), dh(6, 2), joints(6));
        // clang-format on
        return T07;
    }

    Interval compute_psi_limits(uint8_t rconf, const std::vector<Interval> &joint_limits)
    {
        // use bitshift to extract the robot configuration
        const double shoulder = -(rconf >> 2) * 2 + 1; // [1,x,x]->1, [0,x,x]->-1
        // const double elbow = -((rconf >> 1) & 1) * 2 + 1; // [x,1,x]->1, [x,0,x]->-1
        const double wrist = -(rconf & 1) * 2 + 1; // [x,x,1]->1, [x,x,0]->-1

        // Joint 1
        psi_limits[0] = tan_joint_limit(shoulder * As(1, 1),
                                        shoulder * As(0, 1),
                                        shoulder * Bs(1, 1),
                                        shoulder * Bs(0, 1),
                                        shoulder * Cs(1, 1),
                                        shoulder * Cs(0, 1),
                                        joint_limits[0]);
        // Joint 2
        psi_limits[1] = cos_joint_limit(As(2, 1), Bs(2, 1), Cs(2, 1), shoulder, joint_limits[1]);
        // Joint 3
        psi_limits[2] = tan_joint_limit(-shoulder * As(2, 2),
                                        -shoulder * As(2, 0),
                                        -shoulder * Bs(2, 2),
                                        -shoulder * Bs(2, 0),
                                        -shoulder * Cs(2, 2),
                                        -shoulder * Cs(2, 0),
                                        joint_limits[2]);
        // Joint 5
        psi_limits[3] = tan_joint_limit(wrist * Aw(1, 2),
                                        wrist * Aw(0, 2),
                                        wrist * Bw(1, 2),
                                        wrist * Bw(0, 2),
                                        wrist * Cw(1, 2),
                                        wrist * Cw(0, 2),
                                        joint_limits[4]);
        // Joint 6
        psi_limits[4] = cos_joint_limit(Aw(2, 2), Bw(2, 2), Cw(2, 2), wrist, joint_limits[5]);
        // Joint 7
        psi_limits[5] = tan_joint_limit(wrist * Aw(2, 1),
                                        -wrist * Aw(2, 0),
                                        wrist * Bw(2, 1),
                                        -wrist * Bw(2, 0),
                                        wrist * Cw(2, 1),
                                        -wrist * Cw(2, 0),
                                        joint_limits[6]);
        // compute the intersection of the psi limits
        return IntersectIntervals(psi_limits);
    }


private:
    /**
     * @brief Compute the reference plane for the elbow
     * 
     * @warning only call inside the compute method
    */
    void reference_plane(double elbow, Eigen::VectorXd &joints)
    {
        double tol = 1e-6;

        if ((xsw.cross(Eigen::Vector3d(0, 0, 1))).norm() > tol)
        {
            joints(0) = std::atan2(xsw(1), xsw(0));
        }
        else
        {
            joints(0) = 0;
        }

        // Joint 2
        double r = std::hypot(xsw(0), xsw(1));
        double dsw = xsw.norm();
        double phi = std::acos((lse * lse + dsw * dsw - lew * lew) / (2 * lse * dsw));

        joints(1) = std::atan2(r, xsw(2)) + elbow * phi;

        // Lower arm transformation
        pinocchio::SE3 T01 = dh_calc(dh(0, 0), dh(0, 1), dh(0, 2), joints(0));
        pinocchio::SE3 T12 = dh_calc(dh(1, 0), dh(1, 1), dh(1, 2), joints(1));
        pinocchio::SE3 T23 = dh_calc(dh(2, 0), dh(2, 1), dh(2, 2), 0);
        T03_o = T01 * T12 * T23;

        Eigen::Vector3d x0e = (T03_o * T34).translation(); // reference elbow position
        Eigen::Vector3d v1 = (x0e - xs0).normalized();     // unit vector from shoulder to elbow
        Eigen::Vector3d v2 = (xw0 - xs0).normalized();     // unit vector from shoulder to wrist

        ref_plane_vector = v1.cross(v2);
    }

    // Denavit-Hartenberg parameters for the iiwa7
    inline double dh(int i, int j) const
    {
        std::array<std::array<double, 4>, 7> params = {{
            {0, -M_PI / 2, l_iiwa7[0], 0},
            {0, M_PI / 2, 0, 0},
            {0, M_PI / 2, l_iiwa7[1], 0},
            {0, -M_PI / 2, 0, 0},
            {0, -M_PI / 2, l_iiwa7[2], 0},
            {0, M_PI / 2, 0, 0},
            {0, 0, l_iiwa7[3], 0},
        }};
        return params[i][j];
    }

    /**
     * @brief Compute the transformation matrix based on the Denavit-Hartenberg parameters
    */
    inline pinocchio::SE3 dh_calc(double a, double alpha, double d, double theta) const
    {
        pinocchio::SE3 T;
        // clang-format off
        T.translation() << 0, 0, d;
        T.rotation() << cos(theta),     -sin(theta) * cos(alpha),   sin(theta) * sin(alpha),
                        sin(theta),     cos(theta) * cos(alpha),    -cos(theta) * sin(alpha),
                        0,              sin(alpha),                 cos(alpha);
        T.translation() << a * cos(theta), a * sin(theta), d;
        // clang-format on
        return T;
    }

    pinocchio::SE3 T03_o;
    Eigen::Vector3d ref_plane_vector;

    Eigen::Matrix3d R03;
    pinocchio::SE3 T34;
    Eigen::Matrix3d R47;

    // Shoulder matrices
    Eigen::Matrix3d As;
    Eigen::Matrix3d Bs;
    Eigen::Matrix3d Cs;

    // Wrist matrices
    Eigen::Matrix3d Aw;
    Eigen::Matrix3d Bw;
    Eigen::Matrix3d Cw;

    // pose dependent position vectors
    Eigen::Vector3d xend;
    Eigen::Vector3d xw0;
    Eigen::Vector3d xsw;
    Eigen::Vector3d usw;
    Eigen::Matrix3d skew_usw;

    // psi intervals defined by each joint (except joint 4)
    std::vector<Interval> psi_limits;

    // Shoulder position from base
    const Eigen::Vector3d xs0 = Eigen::Vector3d(0, 0, l_iiwa7[0]);
    const Eigen::Vector3d xse = Eigen::Vector3d(0, l_iiwa7[1], 0);
    const Eigen::Vector3d xew = Eigen::Vector3d(0, 0, l_iiwa7[2]);
    // End-effector position from wrist
    const Eigen::Vector3d xwt = Eigen::Vector3d(0, 0, l_iiwa7[3]);

    const double lbs = l_iiwa7[0];
    const double lse = l_iiwa7[1];
    const double lew = l_iiwa7[2];
};

#endif // MARS_MODEL_INVERSE_KINEMATICS_HPP
