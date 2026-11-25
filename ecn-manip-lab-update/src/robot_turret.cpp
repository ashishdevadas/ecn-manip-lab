#include <ecn_manip/robot_turret.h>
#include <ecn_manip/trig_solvers.h>

// Model of Turret robot
// MDH table (models/turret.yml):
//   1: [0,     0, q1, b ]
//   2: [pi/2,  0, q2, 0 ]
//   3: [-pi/2, 0, 0,  q3]   (prismatic)
//   e: [0,     0, 0,  d ]
// b = 0.5 m, d = 0.1 m

// Any constant transform at base or end-effector
void ecn::RobotTurret::init_wMe()
{
    // tool point at d = 0.1 m along z of the last frame
    wMe[2][3] = 0.1;
}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    const double b{0.5};

    const auto c1{cos(q[0])};
    const auto c2{cos(q[1])};
    const auto s1{sin(q[0])};
    const auto s2{sin(q[1])};
    M[0][0] = c1*c2;
    M[0][1] = -s1;
    M[0][2] = -s2*c1;
    M[0][3] = -q[2]*s2*c1;
    M[1][0] = s1*c2;
    M[1][1] = c1;
    M[1][2] = -s1*s2;
    M[1][3] = -q[2]*s1*s2;
    M[2][0] = s2;
    M[2][1] = 0;
    M[2][2] = c2;
    M[2][3] = b + q[2]*c2;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1.;

    return M;
}


// Inverse Geometry
// Only the wrist position can be imposed (3 joints):
//   tx = -q3 s2 c1,  ty = -q3 s2 s1,  tz = b + q3 c2
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    const double b{0.5};
    const auto [tx, ty, tz] = explodeTranslation(Md);

    // q1 from -tx s1 + ty c1 = 0 (2 solutions), undefined on the vertical axis
    std::vector<double> q1s;
    if(isNull(tx) && isNull(ty))
        q1s = {q0[0]};
    else
        for(const double q1: solveType2(-tx, ty, 0))
            q1s.push_back(q1);

    const auto addAll = [&](bool checkPose)
    {
        for(const double q1: q1s)
        {
            const auto [c1, s1] = cos_sin(q1);
            const double rho{c1*tx + s1*ty};
            // -q3 s2 = rho,  q3 c2 = tz - b
            for(const auto &qs: solveType4(-1, rho, 1, tz - b))
            {
                if(checkPose)
                    addCandidate({q1, qs.qi, qs.qj}, Md);    // reaches Md and respects the joint limits
                else
                    addCandidate({q1, qs.qi, qs.qj});        // position only
            }
        }
    };

    // Ex. 3: full check. Ex. 5: intermediate poses on the straight line have an orientation
    // a 3-DOF robot cannot reach, so fall back to the position-only solutions
    addAll(true);
    if(q_candidates.empty())
        addAll(false);

    return bestCandidate(q0);
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);

    const auto c1{cos(q[0])};
    const auto c2{cos(q[1])};
    const auto s1{sin(q[0])};
    const auto s2{sin(q[1])};
    J[0][0] = q[2]*s1*s2;
    J[0][1] = -q[2]*c1*c2;
    J[0][2] = -s2*c1;
    J[1][0] = -q[2]*s2*c1;
    J[1][1] = -q[2]*s1*c2;
    J[1][2] = -s1*s2;
    J[2][0] = 0;
    J[2][1] = -q[2]*s2;
    J[2][2] = c2;
    J[3][0] = 0;
    J[3][1] = s1;
    J[3][2] = 0;
    J[4][0] = 0;
    J[4][1] = -c1;
    J[4][2] = 0;
    J[5][0] = 1.;
    J[5][1] = 0;
    J[5][2] = 0;

    return J;
}
