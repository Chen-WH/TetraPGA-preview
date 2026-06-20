#pragma once

#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
#include <urdf_parser/urdf_parser.h>
#include "TetraPGA/Joints.hpp"
#include "TetraPGA/Motor.hpp"
#include "TetraPGA/PGA.hpp"

namespace TetraPGA {

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& v) {
    using Scalar = typename Derived::Scalar;
    Eigen::Matrix<Scalar, 3, 3> skew_matrix;
    skew_matrix <<  0, -v(2), v(1),
                    v(2), 0, -v(0),
                    -v(1), v(0), 0;
    return skew_matrix;
}

/****** define the struct of robot model ******/
template <typename Scalar = double>
struct Model {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;                                       // Robot model name
    
    int n = 0;                                              // Number of rigid bodies, including the base
    std::vector<char> type;                                 // Joint types, dim = n
    std::vector<int> parent;                                // Parent Indices of each body, dim = n
    std::vector<std::vector<int>> ancestor = {};            // Ancestor Indices of each body, dim = n

    int dof_a = 0;                                          // Degree of freedoms
    VectorXs<Scalar> qa0;                                   // Initial configuration coordinates, dim = dof_a
    std::map<int, std::vector<int>> id_map;                 // Map body id to joint coordinate id, dim = n

    std::vector<Motor3D<Scalar>> Mj;                  // The motor in the local frame of each body, dim = n
    std::vector<Motor3D<Scalar>> M0;                  // The motor in the base frame of each body, dim = n
    std::vector<Line3D<Scalar>> Lj;                   // The joint line in the local frame of each joint, dim = n
    std::vector<Eigen::Matrix<Scalar, 1, 6>> Ljstar;  // The metric of each joint line, dim = n
    std::vector<Line3D<Scalar>> L0;                   // The joint line in the base frame of each joint, dim = n
    
    Line3D<Scalar> gravity;                         // Gravity acceleration
    std::vector<Eigen::Matrix<Scalar, 6, 6>> I;     // Inertia tensor of each rigid body w.r.t. body-fixed frame, dim = n

    VectorXs<Scalar> lowerPositionLimit;            // Lower position limits for each joint, dim = dof_a
    VectorXs<Scalar> upperPositionLimit;            // Upper position limits for each joint, dim = dof_a
    VectorXs<Scalar> velocityLimit;                 // Velocity limits for each joint, dim = dof_a
    VectorXs<Scalar> effortLimit;                   // Effort limits for each joint, dim = dof_a

    int num_collision_ssl = 0;                      // Number of collision capsules
    std::vector<SSL<Scalar>> collisionSSL;          // Collision capsules for each link, defined in the base frame of the link

    /****** Constructors ******/
    // from urdf file
    Model(const std::string& urdf_file);

    // from explicit parameters
    Model(const std::string& model_name,
        const int num_bodies,
        const std::vector<char>& joint_types,
        const std::vector<int>& parent_indices,
        const VectorXs<Scalar> init_q,
        const std::vector<Motor3D<Scalar>>& init_M,
        const std::vector<Line3D<Scalar>>& init_L,
        const Line3D<Scalar>& gravity_acc,
        const std::vector<Scalar>& mass,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& inertia_tensors,
        const std::vector<Eigen::Matrix<Scalar, 3, 1>>& CoMs,
        const VectorXs<Scalar>& lower_position_limit,
        const VectorXs<Scalar>& upper_position_limit,
        const VectorXs<Scalar>& velocity_limit,
        const VectorXs<Scalar>& effort_limit,
        const std::vector<SSL<Scalar>>& collision_ssl
    );
};

template <typename Scalar = double>
struct Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Kinematics
    VectorXs<Scalar> q;   // joint coordinates
    Eigen::Matrix<Scalar, 8, Eigen::Dynamic> Mi;  // configuration of each rigid body, rigid motion from last configuration to the current configuration
    Eigen::Matrix<Scalar, 8, Eigen::Dynamic> M;   // configuration of each rigid body, rigid motion from initial configuration to the current configuration

    // Differential kinematics
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> jac;  // geometric/analytic jacobian
    Eigen::Matrix<Scalar, 8, Eigen::Dynamic> jacM; // motor jacobian
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> L;    // joint axes in the base frame
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> dL;   // time derivative of joint axes in the base frame
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> ddL;  // second time derivative of joint axes in the base frame
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> V;    // spatial velocity
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> dV;   // spatial acceleration
    
    // Inverse Dynamics - RNEA
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> dPi;  // force propagated to parent
    VectorXs<Scalar> tau;   // joint forces/torques
    std::vector<Eigen::Matrix<Scalar, 6, 6>> hI;
    std::vector<Eigen::Matrix<Scalar, 6, 3>> VI;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 6> Lstar;
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> fext_zero;

    // Forward Dynamics - ABA
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> c;
	Eigen::Matrix<Scalar, 6, Eigen::Dynamic> F;
	Eigen::Matrix<Scalar, 6, Eigen::Dynamic> gamma;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 6> gammaT;
    VectorXs<Scalar> d;
	VectorXs<Scalar> u;
    std::vector<Eigen::Matrix<Scalar, 6, 6>> Ia;  // articulated inertia
	VectorXs<Scalar> ddq;

    // First-order derivatives of inverse dynamics
    MatrixXs<Scalar> ptau_pq;   // partial derivative of tau w.r.t q
    MatrixXs<Scalar> ptau_pdq;  // partial derivative of tau w.r.t dq
    MatrixXs<Scalar> ptau_pddq; // partial derivative of tau w.r.t ddq
    std::vector<MatrixXs<Scalar>> p2tau_pqpq;   // second derivative of tau w.r.t. q and q, indexed as [q](tau, q)
    std::vector<MatrixXs<Scalar>> p2tau_pdqpq;  // second derivative of tau w.r.t. dq and q, indexed as [dq](tau, q)
    std::vector<MatrixXs<Scalar>> p2tau_pdqpdq; // second derivative of tau w.r.t. dq and dq, indexed as [dq](tau, dq)
    std::vector<MatrixXs<Scalar>> p2tau_pqpddq; // second derivative of tau w.r.t. q and ddq, indexed as [q](tau, ddq)

    // First-order derivatives of forward dynamics
    MatrixXs<Scalar> pddq_pq;   // partial derivative of ddq w.r.t q
    MatrixXs<Scalar> pddq_pdq;  // partial derivative of ddq w.r.t dq
    MatrixXs<Scalar> pddq_ptau; // partial derivative of ddq w.r.t tau
    std::vector<MatrixXs<Scalar>> p2ddq_pqpq;     // second derivative of ddq w.r.t. q and q, indexed as [q](ddq, q)
    std::vector<MatrixXs<Scalar>> p2ddq_pdqpq;    // second derivative of ddq w.r.t. dq and q, indexed as [q](ddq, dq)
    std::vector<MatrixXs<Scalar>> p2ddq_pdqpdq;   // second derivative of ddq w.r.t. dq and dq, indexed as [dq](ddq, dq)
    std::vector<MatrixXs<Scalar>> p2ddq_ptaupq;   // second derivative of ddq w.r.t. tau and q, indexed as [q](ddq, tau)
    MatrixXs<Scalar> u_aza;
    std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> F2_aza;
    std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> F_aza;
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> hF2_aza;
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> hF_aza;
    std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> dV2_aza;
    std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> dV_aza;

    // Collision detection
    std::vector<Point3D<Scalar>> SSL_A;      // Line point A of the collision capsule in the base frame, used for collision detection
    std::vector<Point3D<Scalar>> SSL_B;      // Line point B of the collision capsule in the base frame, used for collision detection
    std::vector<Eigen::Matrix<Scalar, 4, Eigen::Dynamic>> SSL_jacA;
    std::vector<Eigen::Matrix<Scalar, 4, Eigen::Dynamic>> SSL_jacB;

    // Constructor
    Data(const Model<Scalar>& model);
};

/****** Constructor of Model ******/
template <typename Scalar>
Model<Scalar>::Model(const std::string& urdf_file) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

	auto urdf_model = urdf::parseURDFFile(urdf_file);
	if (!urdf_model) {
	    throw std::invalid_argument("Failed to parse URDF file: " + urdf_file);
	}

	name = urdf_model->getName();
	auto root_link = urdf_model->getRoot();
	if (!root_link) {
	    throw std::invalid_argument("URDF has no root link: " + urdf_file);
	}

    type.clear();
    parent.clear();
    Mj.clear();
    Lj.clear();
    I.clear();
    lowerPositionLimit.resize(0);
    upperPositionLimit.resize(0);
    velocityLimit.resize(0);
    effortLimit.resize(0);
    collisionSSL.clear();
    num_collision_ssl = 0;
    dof_a = 0;
    id_map.clear();

    // Base entry
    type.push_back('\0');
    parent.push_back(0);
    Mj.push_back(Motor3D<Scalar>(Scalar(1), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)));
    Lj.push_back(Line3D<Scalar>(Scalar(0), Scalar(0), Scalar(1), Scalar(0), Scalar(0), Scalar(0)));
    I.push_back(Mat6::Zero());

    std::vector<Scalar> lower_limits;
    std::vector<Scalar> upper_limits;
    std::vector<Scalar> velocity_limits;
    std::vector<Scalar> effort_limits;

    auto urdf_pose_to_rq = [](const urdf::Pose& pose, Vec3& r_out, Eigen::Quaternion<Scalar>& q_out) {
        r_out << static_cast<Scalar>(pose.position.x),
                 static_cast<Scalar>(pose.position.y),
                 static_cast<Scalar>(pose.position.z);
        q_out = Eigen::Quaternion<Scalar>(
            static_cast<Scalar>(pose.rotation.w),
            static_cast<Scalar>(pose.rotation.x),
            static_cast<Scalar>(pose.rotation.y),
            static_cast<Scalar>(pose.rotation.z)
        );
        if (q_out.norm() == Scalar(0)) {
            q_out = Eigen::Quaternion<Scalar>::Identity();
        }
        q_out.normalize();
    };

    auto inertia_link_classical = [&](const urdf::LinkConstSharedPtr& link) {
        Mat6 I_link = Mat6::Zero();
        if (!link || !link->inertial) {
            return I_link;
        }
        const auto& inertial = link->inertial;

        Vec3 r;
        Eigen::Quaternion<Scalar> q;
        urdf_pose_to_rq(inertial->origin, r, q);
        Mat3 R = q.toRotationMatrix();

        const Scalar m = static_cast<Scalar>(inertial->mass);
        Mat3 I_inertial;
        I_inertial << static_cast<Scalar>(inertial->ixx), static_cast<Scalar>(inertial->ixy), static_cast<Scalar>(inertial->ixz),
                      static_cast<Scalar>(inertial->ixy), static_cast<Scalar>(inertial->iyy), static_cast<Scalar>(inertial->iyz),
                      static_cast<Scalar>(inertial->ixz), static_cast<Scalar>(inertial->iyz), static_cast<Scalar>(inertial->izz);

        // Convert CoM inertia to link frame and then apply parallel-axis theorem.
        Mat3 I_com_link = R * I_inertial * R.transpose();
        Mat3 I_origin = I_com_link + m * (r.squaredNorm() * Mat3::Identity() - r * r.transpose());
        Mat3 S = skew(r);

        I_link.template block<3, 3>(0, 0) = I_origin;
        I_link.template block<3, 3>(0, 3) = m * S;
        I_link.template block<3, 3>(3, 0) = -m * S;
        I_link.template block<3, 3>(3, 3) = m * Mat3::Identity();
        return I_link;
    };

    auto adinv_from_Rp = [&](const Mat3& R, const Vec3& p) {
        Mat6 AdInv = Mat6::Zero();
        Mat3 S = skew(p);
        AdInv.template block<3, 3>(0, 0) = R.transpose();
        AdInv.template block<3, 3>(3, 0) = -R.transpose() * S;
        AdInv.template block<3, 3>(3, 3) = R.transpose();
        return AdInv;
    };

    auto axis_line = [](const urdf::JointConstSharedPtr& joint, char t) {
        const Scalar ax = static_cast<Scalar>(joint->axis.x);
        const Scalar ay = static_cast<Scalar>(joint->axis.y);
        const Scalar az = static_cast<Scalar>(joint->axis.z);
        if (t == 'R') {
            return Line3D<Scalar>(ax, ay, az, Scalar(0), Scalar(0), Scalar(0));
        }
        return Line3D<Scalar>(Scalar(0), Scalar(0), Scalar(0), ax, ay, az);
    };

    auto append_limits = [&](const urdf::JointConstSharedPtr& joint, char t) {
        const Scalar inf = std::numeric_limits<Scalar>::infinity();
        const int joint_dof = joint::dof(t, -1, "Model(urdf)::append_limits");
        if (joint_dof > 1) {
            for (int k = 0; k < joint_dof; ++k) {
                lower_limits.push_back(-inf);
                upper_limits.push_back(inf);
                velocity_limits.push_back(inf);
                effort_limits.push_back(inf);
            }
            return;
        }

        if (t == 'R' && joint->type == urdf::Joint::CONTINUOUS) {
            lower_limits.push_back(static_cast<Scalar>(-2.0 * M_PI));
            upper_limits.push_back(static_cast<Scalar>(2.0 * M_PI));
        } else if (joint->limits) {
            lower_limits.push_back(static_cast<Scalar>(joint->limits->lower));
            upper_limits.push_back(static_cast<Scalar>(joint->limits->upper));
        } else {
            lower_limits.push_back(-inf);
            upper_limits.push_back(inf);
        }

        if (joint->limits) {
            velocity_limits.push_back(static_cast<Scalar>(joint->limits->velocity));
            effort_limits.push_back(static_cast<Scalar>(joint->limits->effort));
        } else {
            velocity_limits.push_back(inf);
            effort_limits.push_back(inf);
        }
    };

    std::function<void(const urdf::LinkConstSharedPtr&, int, const Mat3&, const Vec3&)> dfs;
    dfs = [&](const urdf::LinkConstSharedPtr& link, int current_body, const Mat3& R_body_link, const Vec3& p_body_link) {
        if (!link) {
            return;
        }

        // Merge current link inertia into current kept body (fixed-joint chain support).
        const Mat6 I_link = inertia_link_classical(link);
        const Mat6 AdInv = adinv_from_Rp(R_body_link, p_body_link);
        Mat6 I_body = I[current_body];
        I_body += AdInv.transpose() * I_link * AdInv;
        I[current_body] = I_body;

        for (const auto& joint : link->child_joints) {
            if (!joint) {
                continue;
            }
            auto child_link = urdf_model->getLink(joint->child_link_name);
            if (!child_link) {
                continue;
            }

            Vec3 r_pc;
            Eigen::Quaternion<Scalar> q_pc;
            urdf_pose_to_rq(joint->parent_to_joint_origin_transform, r_pc, q_pc);
            const Mat3 R_pc = q_pc.toRotationMatrix();

            // Compose kept-body -> child-link transform through possibly collapsed fixed ancestors.
            const Mat3 R_body_child = R_body_link * R_pc;
            const Vec3 p_body_child = p_body_link + R_body_link * r_pc;

            char jt = 'f';
            switch (joint->type) {
                case urdf::Joint::REVOLUTE: jt = 'R'; break;
                case urdf::Joint::CONTINUOUS: jt = 'R'; break;
                case urdf::Joint::PRISMATIC: jt = 'P'; break;
                case urdf::Joint::PLANAR: jt = joint::kPlanarRoot; break;
                case urdf::Joint::FLOATING: jt = 'F'; break;
                case urdf::Joint::FIXED: jt = 'f'; break;
                default: jt = 'f'; break;
            }

            if (jt == 'f') {
                dfs(child_link, current_body, R_body_child, p_body_child);
                continue;
            }

            const int new_body = static_cast<int>(type.size());
            type.push_back(jt);
            parent.push_back(current_body);
            Mj.push_back(Quat_2motor(Eigen::Quaternion<Scalar>(R_body_child), p_body_child));
            I.push_back(Mat6::Zero());

            if (joint::isRootOnly(jt)) {
                Lj.push_back(Line3D<Scalar>(Scalar(0), Scalar(0), Scalar(1), Scalar(0), Scalar(0), Scalar(0)));
            } else {
                Lj.push_back(axis_line(joint, jt));
            }
            append_limits(joint, jt);

            // New kept body frame is this child link frame.
            dfs(child_link, new_body, Mat3::Identity(), Vec3::Zero());
        }
    };

    dfs(root_link, 0, Mat3::Identity(), Vec3::Zero());

    n = static_cast<int>(type.size());
    M0.resize(n);
    Ljstar.resize(n);
    L0.resize(n);
    M0[0] = Mj[0];
    Ljstar[0] = ga_metric(Lj[0]);
    L0[0] = ga_rbm(M0[0], Lj[0]);
    for (int i = 1; i < n; ++i) {
        M0[i] = ga_mul(Mj[i], M0[parent[i]]);
        Ljstar[i] = ga_metric(Lj[i]);
        L0[i] = ga_rbm(M0[i], Lj[i]);
    }

    // Convert classical spatial inertia layout to TetraPGA layout.
    for (int i = 0; i < n; ++i) {
        const Mat3 A = I[i].template block<3, 3>(0, 0);
        const Mat3 C = I[i].template block<3, 3>(3, 0);
        const Mat3 D = I[i].template block<3, 3>(3, 3);
        Mat6 I_ga = Mat6::Zero();
        I_ga.template block<3, 3>(0, 0) = C;
        I_ga.template block<3, 3>(0, 3) = D;
        I_ga.template block<3, 3>(3, 0) = A;
        I_ga.template block<3, 3>(3, 3) = -C;
        I[i] = I_ga;
    }

    // Build dof map and dof count.
    int q_offset = 0;
    dof_a = 0;
    id_map.clear();
    for (int i = 1; i < n; ++i) {
        const int joint_dof = joint::dof(type[i], i, "Model(urdf)");
        joint::validateRootPlacement(type[i], parent[i], i, "Model(urdf)");
        id_map[i].clear();
        id_map[i].reserve(static_cast<std::size_t>(joint_dof));
        for (int j = 0; j < joint_dof; ++j) {
            id_map[i].push_back(q_offset + j);
        }
        q_offset += joint_dof;
        dof_a += joint_dof;
	}

    qa0.resize(dof_a);
    qa0.setZero();

    lowerPositionLimit.resize(dof_a);
    upperPositionLimit.resize(dof_a);
    velocityLimit.resize(dof_a);
    effortLimit.resize(dof_a);
    if (static_cast<int>(lower_limits.size()) == dof_a) {
        for (int i = 0; i < dof_a; ++i) {
            lowerPositionLimit(i) = lower_limits[i];
            upperPositionLimit(i) = upper_limits[i];
            velocityLimit(i) = velocity_limits[i];
            effortLimit(i) = effort_limits[i];
        }
    } else {
        lowerPositionLimit.setConstant(-std::numeric_limits<Scalar>::infinity());
        upperPositionLimit.setConstant(std::numeric_limits<Scalar>::infinity());
        velocityLimit.setConstant(std::numeric_limits<Scalar>::infinity());
        effortLimit.setConstant(std::numeric_limits<Scalar>::infinity());
    }

    gravity = Line3D<Scalar>::Zero();

    ancestor.resize(n);
    for (int i = 0; i < n; ++i) {
        std::vector<int> temp_vec;
        int k = i;
        while (k > 0 && parent[k] > 0) {
            temp_vec.insert(temp_vec.begin(), parent[k]);
            k = parent[k];
        }
        ancestor[i] = temp_vec;
    }
}

/****** Constructor of Model ******/
template <typename Scalar>
Model<Scalar>::Model(const std::string& model_name,
    const int num_bodies,
    const std::vector<char>& joint_types,
    const std::vector<int>& parent_indices,
    const VectorXs<Scalar> init_q,
    const std::vector<Motor3D<Scalar>>& init_M,
    const std::vector<Line3D<Scalar>>& init_L,
    const Line3D<Scalar>& gravity_acc,
    const std::vector<Scalar>& mass,
    const std::vector<Eigen::Matrix<Scalar, 6, 1>>& inertia_tensors,
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& CoMs,
    const VectorXs<Scalar>& lower_position_limit,
    const VectorXs<Scalar>& upper_position_limit,
    const VectorXs<Scalar>& velocity_limit,
    const VectorXs<Scalar>& effort_limit,
    const std::vector<SSL<Scalar>>& collision_ssl
) :
    name(model_name),
    n(num_bodies),
    type(joint_types),
    parent(parent_indices),
    qa0(init_q),
    Mj(init_M),
    Lj(init_L),
    gravity(gravity_acc),
    lowerPositionLimit(lower_position_limit),
    upperPositionLimit(upper_position_limit),
    velocityLimit(velocity_limit),
    effortLimit(effort_limit),
    collisionSSL(collision_ssl)
{
    // Generate ancestor indices from parent indices
    ancestor.resize(n);
    for (int i = 0; i < n; ++i) {
        std::vector<int> temp_vec;
        int k = i;
        while (k > 0 && parent[k] > 0) {
            temp_vec.insert(temp_vec.begin(), parent[k]);
            k = parent[k];
        }
        ancestor[i] = temp_vec;
    }
    // Construct id_map and allocate M0/L0/I base entries
    int q_offset = 0;
    id_map.clear();
    for (int i = 1; i < n; ++i) {
        const int joint_dof = joint::dof(type[i], i, "Model");
        joint::validateRootPlacement(type[i], parent[i], i, "Model");
        id_map[i].clear();
        id_map[i].reserve(static_cast<std::size_t>(joint_dof));
        for (int j = 0; j < joint_dof; ++j) {
            id_map[i].push_back(q_offset + j);
        }
        q_offset += joint_dof;
        dof_a += joint_dof;
	}
    
    // Calculate motors in base frame and joint axes in base frame
    // Ensure vectors sized before assignment
    M0.resize(n);
    Ljstar.resize(n);
    L0.resize(n);
    M0[0] = Mj[0];
    Ljstar[0] = ga_metric(Lj[0]);
    L0[0] = ga_rbm(M0[0], Lj[0]);
    for (int i = 1; i < n; ++i) {
        M0[i] = ga_mul(Mj[i], M0[parent[i]]);
        Ljstar[i] = ga_metric(Lj[i]);
        L0[i] = ga_rbm(M0[i], Lj[i]);
    }
    
    // calculate  dynamic parameters in the joint frame
    I.resize(n);
    for (int i = 0; i < n; ++i) {
        Scalar m = mass[i];
        Eigen::Matrix<Scalar, 3, 3> Ic;
        Ic << inertia_tensors[i](0), inertia_tensors[i](1), inertia_tensors[i](2),
              inertia_tensors[i](1), inertia_tensors[i](3), inertia_tensors[i](4),
              inertia_tensors[i](2), inertia_tensors[i](4), inertia_tensors[i](5);
        
        Eigen::Matrix<Scalar, 3, 3> Rc = skew(CoMs[i]);
        Eigen::Matrix<Scalar, 3, 3> J = Ic - m * Rc * Rc;
        
        // I = [-m*Rc, m*I3; J, m*Rc]
        I[i].setZero();
        I[i].template block<3, 3>(0, 0) = -m * Rc;
        I[i].template block<3, 3>(0, 3) = m * Eigen::Matrix<Scalar, 3, 3>::Identity();
        I[i].template block<3, 3>(3, 0) = J;
        I[i].template block<3, 3>(3, 3) = m * Rc;
    }

    //calculate the number of collision capsules
    num_collision_ssl = collision_ssl.size();

    for (int i = 0; i < num_collision_ssl; ++i) {
        collisionSSL[i].endpointA = pga_rbm3(M0[collisionSSL[i].id], collisionSSL[i].endpointA);
        collisionSSL[i].endpointB = pga_rbm3(M0[collisionSSL[i].id], collisionSSL[i].endpointB);
    }
};

/****** Constructor of Data ******/
template <typename Scalar>
Data<Scalar>::Data(const Model<Scalar>& model) {
    q.resize(model.dof_a);
    q.setZero();
    
    Mi.resize(8, model.n);
    Mi.setZero();
    Mi.col(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    M.resize(8, model.n);
    M.setZero();
    M.col(0) = model.M0[0];

    jac.resize(6, model.dof_a);
    jac.setZero();
    jacM.resize(8, model.dof_a);
    jacM.setZero();

    L.resize(6, model.n);
    L.setZero();
    dL.resize(6, model.n);
    dL.setZero();
    ddL.resize(6, model.n);
    ddL.setZero();

    V.resize(6, model.n);
    V.setZero();
    dV.resize(6, model.n);
    dV.setZero();
    dV.col(0) = -model.gravity;

    dPi.resize(6, model.n);
    dPi.setZero();
    tau.resize(model.dof_a);
    tau.setZero();
    hI.resize(model.n);
    VI.resize(model.n);
    for (int i = 0; i < model.n; ++i) {
        hI[i].setZero();
        VI[i].setZero();
    }
    Lstar.resize(model.n, 6);
    Lstar.setZero();
    fext_zero.resize(6, model.n);
    fext_zero.setZero();

    c.resize(6, model.n);
    c.setZero();
	F.resize(6, model.n);
    F.setZero();
	gamma.resize(6, model.n);
    gamma.setZero();
    gammaT.resize(model.n, 6);
    gammaT.setZero();
    d.resize(model.n);
    d.setZero();
	u.resize(model.n);
    u.setZero();
    Ia.resize(model.n);
    for (int i = 0; i < model.n; ++i) {
        Ia[i].setZero();
    }
	ddq.resize(model.dof_a);
    ddq.setZero();

    ptau_pq.resize(model.dof_a, model.dof_a);
    ptau_pq.setZero();
    ptau_pdq.resize(model.dof_a, model.dof_a);
    ptau_pdq.setZero();
    ptau_pddq.resize(model.dof_a, model.dof_a);
    ptau_pddq.setZero();
    p2tau_pqpq.resize(model.dof_a);
    p2tau_pdqpq.resize(model.dof_a);
    p2tau_pdqpdq.resize(model.dof_a);
    p2tau_pqpddq.resize(model.dof_a);
    for (int i = 0; i < model.dof_a; ++i) {
        p2tau_pqpq[i].resize(model.dof_a, model.dof_a);
        p2tau_pqpq[i].setZero();
        p2tau_pdqpq[i].resize(model.dof_a, model.dof_a);
        p2tau_pdqpq[i].setZero();
        p2tau_pdqpdq[i].resize(model.dof_a, model.dof_a);
        p2tau_pdqpdq[i].setZero();
        p2tau_pqpddq[i].resize(model.dof_a, model.dof_a);
        p2tau_pqpddq[i].setZero();
    }

    pddq_pq.resize(model.dof_a, model.dof_a);
    pddq_pq.setZero();
    pddq_pdq.resize(model.dof_a, model.dof_a);
    pddq_pdq.setZero();
    pddq_ptau.resize(model.dof_a, model.dof_a);
    pddq_ptau.setZero();
    p2ddq_pqpq.resize(model.dof_a);
    p2ddq_pdqpq.resize(model.dof_a);
    p2ddq_pdqpdq.resize(model.dof_a);
    p2ddq_ptaupq.resize(model.dof_a);
    for (int i = 0; i < model.dof_a; ++i) {
        p2ddq_pqpq[i].resize(model.dof_a, model.dof_a);
        p2ddq_pqpq[i].setZero();
        p2ddq_pdqpq[i].resize(model.dof_a, model.dof_a);
        p2ddq_pdqpq[i].setZero();
        p2ddq_pdqpdq[i].resize(model.dof_a, model.dof_a);
        p2ddq_pdqpdq[i].setZero();
        p2ddq_ptaupq[i].resize(model.dof_a, model.dof_a);
        p2ddq_ptaupq[i].setZero();
    }

    u_aza.resize(model.dof_a, 2 * model.dof_a);
    u_aza.setZero();
    hF2_aza.resize(6, 2 * model.dof_a);
    hF2_aza.setZero();
    hF_aza.resize(6, model.dof_a);
    hF_aza.setZero();
    F2_aza.resize(model.n);
    F_aza.resize(model.n);
    dV2_aza.resize(model.n);
    dV_aza.resize(model.n);
    for (int i = 0; i < model.n; ++i) {
        F2_aza[i].resize(6, 2 * model.dof_a);
        F2_aza[i].setZero();
        F_aza[i].resize(6, model.dof_a);
        F_aza[i].setZero();
        dV2_aza[i].resize(6, 2 * model.dof_a);
        dV2_aza[i].setZero();
        dV_aza[i].resize(6, model.dof_a);
        dV_aza[i].setZero();
    }

    SSL_A.resize(model.num_collision_ssl);
    for (int i = 0; i < model.num_collision_ssl; ++i) {
        SSL_A[i].setZero();
    }
    SSL_B.resize(model.num_collision_ssl);
    for (int i = 0; i < model.num_collision_ssl; ++i) {
        SSL_B[i].setZero();
    }
    SSL_jacA.resize(model.num_collision_ssl);
    SSL_jacB.resize(model.num_collision_ssl);
    for (int i = 0; i < model.num_collision_ssl; ++i) {
        SSL_jacA[i].resize(4, model.dof_a);
        SSL_jacA[i].setZero();
        SSL_jacB[i].resize(4, model.dof_a);
        SSL_jacB[i].setZero();
    }
};

}  // namespace TetraPGA
