#include <ecn_manip/robot_init.h>

using namespace std;
using namespace ecn;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    const auto robot{initRobot(argc, argv, 100)};
    const unsigned n{robot->getDofs()};

    // robot properties - max velocity and acceleration
    const auto vMax{robot->vMax()};
    const auto aMax{robot->aMax()};

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);       // joint position setpoint for initial and final poses
    double t0{0}, tf{1};

    // main control loop
    while(robot->ok())
    {

        // current time [s]
        const auto t{robot->time()};

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            // starting time of this motion [s]
            t0 = t;
        }

        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ControlMode::POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            robot->checkPose(M);
        }


        else if(robot->mode() == ControlMode::VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->guiVelocityScrew();

            vpMatrix R(6, 6);
            putAt(R, M.getRotationMatrix(), 0, 0);
            putAt(R, M.getRotationMatrix(), 3, 3);

            vCommand = robot->fJe(q).pseudoInverse() * R * v;

            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ControlMode::DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }


        else if(robot->mode() == ControlMode::POLYNOM_P2P)
        {
            // reach Md with polynomial joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);

                // minimum time so that no joint exceeds vMax or aMax
                tf = 0.01;
                for(unsigned int i = 0; i < n; ++i)
                {
                    const double D{std::abs(qf[i] - q0[i])};
                    tf = std::max(tf, 15.*D/(8.*vMax[i]));
                    tf = std::max(tf, sqrt(10.*D/(sqrt(3.)*aMax[i])));
                }
            }

            // p(x) = 10x^3 - 15x^4 + 6x^5, with x = (t - t0)/tf between 0 and 1
            const double x{std::min(1., (t - t0)/tf)};
            const double pt{10*pow(x,3) - 15*pow(x,4) + 6*pow(x,5)};
            qCommand = q0 + pt*(qf - q0);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;

            // 1. progress of the motion: 0 at the start, 1 at the end
            double alpha = (t - t0) / tf;
            alpha = std::min(1., std::max(0., alpha));   // keep it between 0 and 1

            // 2. pose on the straight line between M0 and Md
            Mi = robot->intermediaryPose(M0, Md, alpha);

            // 3. joints that reach this pose (search starts from the current q)
            qCommand = robot->inverseGeometry(Mi, q);


            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::VELOCITY_P2P)
        {
            // go to Md using operational velocity

            // 1. pose error expressed in the desired frame: e*Me = fMe*^-1 . fMe
            const vpHomogeneousMatrix eMe = Md.inverse() * M;
            vpPoseVector pe;
            pe.buildFrom(eMe);                                   // (tx, ty, tz, theta.ux, theta.uy, theta.uz)
            const vpTranslationVector te(pe[0], pe[1], pe[2]);   //
            const vpThetaUVector tu(pe[3], pe[4], pe[5]);        //

            // 2. desired twist in the fixed frame
            const double lambda{robot->lambda()};                // gain from the GUI slider
            const vpTranslationVector vlin = Md.getRotationMatrix() * te * (-lambda);      // v = -lambda Rd t
            const vpColVector w = M.getRotationMatrix() * vpColVector(tu) * (-lambda);     // w = -lambda R thetau

            vpColVector twist(6);
            for(int i = 0; i < 3; ++i)
            {
                twist[i]   = vlin[i];    // linear part
                twist[i+3] = w[i];       // angular part
            }

            // 3. joint velocities with the pseudo-inverse of the Jacobian
            vCommand = robot->fJe(q).pseudoInverse() * twist;
            robot->setJointVelocity(vCommand);
        }


    }
}
