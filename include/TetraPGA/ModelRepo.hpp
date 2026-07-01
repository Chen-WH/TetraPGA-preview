#pragma once

/****** 
 * 
 * Some commonly used robot models  
 * 
 *******/

#include "TetraPGA/Models.hpp"
#include <stdexcept>

namespace TetraPGA {

/****** JAKA Zu12 ******/ 
inline Model<double> jaka(const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    std::string model_name = "jaka_zu12";
    int num_bodies = 7;
    std::vector<char> joint_types = { '\0', 'R', 'R', 'R', 'R', 'R', 'R' };
    std::vector<int> parent_indices = { 0, 0, 1, 2, 3, 4, 5 }; // Parent body indices
    Eigen::Matrix<double, Eigen::Dynamic, 1> init_q(6);
    init_q << 0, M_PI_2, 0, M_PI_2, 0, 0;

    std::vector<Motor3D<double>> Mj(num_bodies);
    Mj[0] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0.0, 0), true);
    Mj[1] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0.10265, 0), true);
    Mj[2] = ga_DH2motor<double>(Eigen::Vector4d(0, M_PI_2, 0, 0), true);
    Mj[3] = ga_DH2motor<double>(Eigen::Vector4d(0.595, 0, 0, 0), true);
    Mj[4] = ga_DH2motor<double>(Eigen::Vector4d(0.5715, 0, -0.1315, 0), true);
    Mj[5] = ga_DH2motor<double>(Eigen::Vector4d(0, M_PI_2, 0.115, 0), true);
    Mj[6] = ga_DH2motor<double>(Eigen::Vector4d(0, -M_PI_2, 0.1035, 0), true);
    std::vector<Line3D<double>> Lj(num_bodies, Line3D<double>(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

    Line3D<double> gravity = Line3D<double>::Zero();
    gravity.segment<3>(3) = g;

    std::vector<double> mass(num_bodies);
    mass = {0.8, 3.78, 12.44, 5.06, 0.883, 1.0, 0.22};

    std::vector<Eigen::Matrix<double, 6, 1>> inertia_tensors(num_bodies);
    inertia_tensors[0] << 0.0102472789321389, -1.25211096927899E-05, -2.032215901439E-07, 0.00775885552091728, 4.74326539193397E-05, 0.0171648317580214;
    inertia_tensors[1] << 0.0404051529004393, -5.35916937084468e-05, 8.56853276426298e-05, 0.0404457306741405, -0.00039414179882763, 0.0343161527751278;
    inertia_tensors[2] << 0.116456439495809, -0.00156765863153774, -3.42571922960543e-06, 1.66880550591637, -1.42545474100985e-05, 1.65302926902866;
    inertia_tensors[3] << 0.0212898342592158, -1.15196860083347e-05, -0.00344346825692792, 0.497633875998665, -6.35439617896331e-07, 0.498513225896089;
    inertia_tensors[4] << 0.00458464465961989, 2.63659533420518e-06, -1.52605286509264e-05, 0.00363879515047717, -3.19585101806877e-05, 0.00455871160423887;
    inertia_tensors[5] << 0.00702353102292446, -2.72522678679305e-06, 1.52464540194323e-05, 0.00413469714863856, 2.81695088352211e-05, 0.00699581168456812;
    inertia_tensors[6] << 0.000597975009893646, -2.41634474044304e-05, -6.20059067536115e-06, 0.000647976470675928, -6.92746756534325e-07, 0.00106463342891045;

    std::vector<Eigen::Matrix<double, 3, 1>> CoMs(num_bodies);
    CoMs[0] << 3.01643284384016E-05, -0.000681463570921656, -0.0225912828938222;
    CoMs[1] << -0.000328431560018958, 0.00406089906856803, -0.0250109305609668;
    CoMs[2] << 0.297499939118518, -1.25165127429216e-08, -0.166069989366573;
    CoMs[3] << 0.294233782543355, -4.72319252706188e-06, -0.0241861096678911;
    CoMs[4] << 4.22017696188881e-06, -0.0150343523848632, 0.00216593552372366;
    CoMs[5] << 3.86594587120648e-06, 0.00426530078104992, -0.00184283112443297;
    CoMs[6] << -0.000786084663781494, 3.88801034911165e-05, -0.0163246941858815;

    Eigen::Matrix<double, Eigen::Dynamic, 1> lower_position_limit(6);
    lower_position_limit << -6.28, -1.48, -3.05, -1.48, -6.28, -1.57;

    Eigen::Matrix<double, Eigen::Dynamic, 1> upper_position_limit(6);
    upper_position_limit << 6.28, 4.62, 3.05, 4.62, 6.28, 1.57;

    Eigen::Matrix<double, Eigen::Dynamic, 1> velocity_limit(6);
    velocity_limit << 0.5*M_PI, 0.5*M_PI, 0.5*M_PI, 0.5*M_PI, 0.5*M_PI, 0.5*M_PI;

    Eigen::Matrix<double, Eigen::Dynamic, 1> effort_limit(6);
    effort_limit << 330, 330, 150, 56, 56, 56;

    std::vector<SSL<double>> collision_ssl;

    return Model<double>(model_name, num_bodies, joint_types, parent_indices, init_q, Mj, Lj, gravity, mass,
                         inertia_tensors, CoMs, lower_position_limit, upper_position_limit, velocity_limit,
                         effort_limit, collision_ssl);
}

/****** UR10 ******/ 
inline Model<double> ur(const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    std::string model_name = "ur10";
    int num_bodies = 7;
    std::vector<char> joint_types = { '\0', 'R', 'R', 'R', 'R', 'R', 'R' };
    std::vector<int> parent_indices = { 0, 0, 1, 2, 3, 4, 5 }; // Parent body indices
    Eigen::Matrix<double, Eigen::Dynamic, 1> init_q(6);
    init_q << 0, -M_PI_2, 0, -M_PI_2, 0, 0;

    std::vector<Motor3D<double>> Mj(num_bodies);
    Mj[0] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0.0, M_PI), true);
    Mj[1] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0.1273, 0), true);
    Mj[2] = ga_DH2motor<double>(Eigen::Vector4d(0, M_PI_2, 0, 0), true);
    Mj[3] = ga_DH2motor<double>(Eigen::Vector4d(-0.612, 0, 0, 0), true);
    Mj[4] = ga_DH2motor<double>(Eigen::Vector4d(-0.5723, 0, 0.163941, 0), true);
    Mj[5] = ga_DH2motor<double>(Eigen::Vector4d(0, M_PI_2, 0.1157, 0), true);
    Mj[6] = ga_DH2motor<double>(Eigen::Vector4d(0, -M_PI_2, 0.0922, 0), true);
    std::vector<Line3D<double>> Lj(num_bodies, Line3D<double>(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

    Line3D<double> gravity = Line3D<double>::Zero();
    gravity.segment<3>(3) = g;

    std::vector<double> mass(num_bodies);
    mass = {4.0, 7.1, 12.7, 4.27, 2, 2, 0.365};

    std::vector<Eigen::Matrix<double, 6, 1>> inertia_tensors(num_bodies);
    inertia_tensors[0] << 0.0061063308908, 0.0, 0.0, 0.0061063308908, 0.0, 0.01125;
    inertia_tensors[1] << 0.03408, 0.00425,  0.00002, 0.02156, -0.00008, 0.03529;
    inertia_tensors[2] << 0.02814, 0.00005, -0.01561, 0.77068,  0.00002, 0.76943;
    inertia_tensors[3] << 0.01014, 0.00008,  0.00916, 0.30928,      0.0, 0.30646;
    inertia_tensors[4] << 0.00296,     0.0, -0.00001, 0.00258,  0.00024, 0.00222;
    inertia_tensors[5] << 0.00296,     0.0,  0.00001, 0.00258,  0.00024, 0.00222;
    inertia_tensors[6] << 0.0004,     0.0,      0.0, 0.00041,      0.0, 0.00034;

    std::vector<Eigen::Matrix<double, 3, 1>> CoMs(num_bodies);
    CoMs[0] << 0.000,  0.000, 0.000;
    CoMs[1] << 0.021, -0.027,  0.000;
    CoMs[2] << -0.232,  0.000,  0.158;
    CoMs[3] << -0.3323,  0.000,  0.068;
    CoMs[4] << 0.000, -0.018,  0.007;
    CoMs[5] << 0.000,  0.018, -0.007;
    CoMs[6] << 0.000,  0.000, -0.026;

    Eigen::Matrix<double, Eigen::Dynamic, 1> lower_position_limit(6);
    lower_position_limit << -2*M_PI, -2*M_PI, -M_PI, -2*M_PI, -2*M_PI, -M_PI_2;

    Eigen::Matrix<double, Eigen::Dynamic, 1> upper_position_limit(6);
    upper_position_limit << 2*M_PI, 2*M_PI, M_PI, 2*M_PI, 2*M_PI, M_PI_2;

    Eigen::Matrix<double, Eigen::Dynamic, 1> velocity_limit(6);
    velocity_limit << 2*M_PI/3, 2*M_PI/3, M_PI, M_PI, M_PI, M_PI;

    Eigen::Matrix<double, Eigen::Dynamic, 1> effort_limit(6);
    effort_limit << 330, 330, 150, 56, 56, 56;

    /*std::vector<SSL<double>> collision_ssl(1);
    collision_ssl[0].id = 5;
    collision_ssl[0].radius = 0.045;
    collision_ssl[0].endpointA << 0.0, -0.03, 0.0, 1.0;
    collision_ssl[0].endpointB << 0.0, 0.06, 0.0, 1.0;*/

    std::vector<SSL<double>> collision_ssl(4);

    collision_ssl[0].id = 2;
    collision_ssl[0].radius = 0.065;
    collision_ssl[0].endpointA << -0.62, 0.0, 0.175, 1.0;
    collision_ssl[0].endpointB << 0.02, 0.0, 0.175, 1.0;

    collision_ssl[1].id = 3;
    collision_ssl[1].radius = 0.055;
    collision_ssl[1].endpointA << -0.58, 0.0, 0.05, 1.0;
    collision_ssl[1].endpointB << 0.02, 0.0, 0.05, 1.0;

    collision_ssl[2].id = 4;
    collision_ssl[2].radius = 0.045;
    collision_ssl[2].endpointA << 0.0, 0.02, 0.0, 1.0;
    collision_ssl[2].endpointB << 0.0, -0.03, 0.0, 1.0;

    collision_ssl[3].id = 5;
    collision_ssl[3].radius = 0.045;
    collision_ssl[3].endpointA << 0.0, -0.03, 0.0, 1.0;
    collision_ssl[3].endpointB << 0.0, 0.06, 0.0, 1.0;

    return Model<double>(model_name, num_bodies, joint_types, parent_indices, init_q, Mj, Lj, gravity, mass,
                         inertia_tensors, CoMs, lower_position_limit, upper_position_limit, velocity_limit,
                         effort_limit, collision_ssl);
}

/****** Franka Emika Panda ******/
inline Model<double> franka(const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    std::string model_name = "franka_emika_panda";
    int num_bodies = 8;
    std::vector<char> joint_types = { '\0', 'R', 'R', 'R', 'R', 'R', 'R', 'R' };
    std::vector<int> parent_indices = { 0, 0, 1, 2, 3, 4, 5, 6 };
    Eigen::Matrix<double, Eigen::Dynamic, 1> init_q(7);
    init_q.setZero();

    std::vector<Motor3D<double>> Mj(num_bodies);
    Mj[0] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0.0, 0), true);
    Mj[1] = ga_DH2motor<double>(Eigen::Vector4d(0.0, 0.0, 0.333, 0.0), true);
    Mj[2] = ga_DH2motor<double>(Eigen::Vector4d(0.0, -M_PI_2, 0.0, 0.0), true);
    Mj[3] = ga_DH2motor<double>(Eigen::Vector4d(0.0, M_PI_2, 0.316, 0.0), true);
    Mj[4] = ga_DH2motor<double>(Eigen::Vector4d(0.0825, M_PI_2, 0.0, 0.0), true);
    Mj[5] = ga_DH2motor<double>(Eigen::Vector4d(-0.0825, -M_PI_2, 0.384, 0.0), true);
    Mj[6] = ga_DH2motor<double>(Eigen::Vector4d(0.0, M_PI_2, 0.0, 0.0), true);
    Mj[7] = ga_mul(ga_DH2motor<double>(Eigen::Vector4d(0.0, 0.0, 0.107, 0.0), true),
        ga_DH2motor<double>(Eigen::Vector4d(0.088, M_PI_2, 0.0, 0.0), true));
    std::vector<Line3D<double>> Lj(num_bodies, Line3D<double>(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

    Line3D<double> gravity = Line3D<double>::Zero();
    gravity.segment<3>(3) = g;

    std::vector<double> mass(num_bodies, 0.0);
    mass = {0.629769273993887, 4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 7.35522e-01};

    std::vector<Eigen::Matrix<double, 6, 1>> inertia_tensors(num_bodies, Eigen::Matrix<double, 6, 1>::Zero());
    inertia_tensors[0] << 0.0031531502307724, 8.29043977620386E-07, 0.000153878135879635, 0.00388160500528917, 8.22996985150111E-06, 0.00428506837339653;
    inertia_tensors[1] << 0.70337, -0.00013900, 0.0067720, 0.70661, 0.019169, 0.0091170;
    inertia_tensors[2] << 0.0079620, -3.9250e-3, 1.0254e-02, 2.8110e-02, 7.0400e-04, 2.5995e-02;
    inertia_tensors[3] << 3.7242e-02, -4.7610e-03, -1.1396e-02, 3.6155e-02, -1.2805e-02, 1.0830e-02;
    inertia_tensors[4] << 2.5853e-02, 7.7960e-03, -1.3320e-03, 1.9552e-02, 8.6410e-03, 2.8323e-02;
    inertia_tensors[5] << 3.5549e-02, -2.1170e-03, -4.0370e-03, 2.9474e-02, 2.2900e-04, 8.6270e-03;
    inertia_tensors[6] << 1.9640e-03, 1.0900e-04, -1.1580e-03, 4.3540e-03, 3.4100e-04, 5.4330e-03;
    inertia_tensors[7] << 1.2516e-02, -4.2800e-04, -1.1960e-03, 1.0027e-02, -7.4100e-04, 4.8150e-03;

    std::vector<Eigen::Matrix<double, 3, 1>> CoMs(num_bodies, Eigen::Matrix<double, 3, 1>::Zero());
    CoMs[0] << -0.0410181918537986, -0.000143266349590146, 0.0499742749991159;
    CoMs[1] << 0.003875, 0.002081, -0.04762;
    CoMs[2] << -0.003141, -0.02872, 0.003495;
    CoMs[3] << 2.7518e-02, 3.9252e-02, -6.6502e-02;
    CoMs[4] << -5.317e-02, 1.04419e-01, 2.7454e-02;
    CoMs[5] << -1.1953e-02, 4.1065e-02, -3.8437e-02;
    CoMs[6] << 6.0149e-02, -1.4117e-02, -1.0517e-02;
    CoMs[7] << 1.0517e-02, -4.252e-03, 6.1597e-02 - 0.107;

    Eigen::Matrix<double, Eigen::Dynamic, 1> lower_position_limit(7);
    lower_position_limit << -2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671;

    Eigen::Matrix<double, Eigen::Dynamic, 1> upper_position_limit(7);
    upper_position_limit << 2.9671, 1.8326, 2.9671, -0.4, 2.9671, 3.8223, 2.9671;

    Eigen::Matrix<double, Eigen::Dynamic, 1> velocity_limit(7);
    velocity_limit << 2.5, 2.5, 2.5, 2.5, 3.0, 3.0, 3.0;

    Eigen::Matrix<double, Eigen::Dynamic, 1> effort_limit(7);
    effort_limit << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;

    std::vector<SSL<double>> collision_ssl;

    return Model<double>(model_name, num_bodies, joint_types, parent_indices, init_q, Mj, Lj, gravity, mass,
                         inertia_tensors, CoMs, lower_position_limit, upper_position_limit, velocity_limit,
                         effort_limit, collision_ssl);
}

/****** URDF-backed models ******/
inline Model<double> leap_hand(const std::string& urdf_file) {
    return Model<double>(urdf_file);
}

inline Model<double> stanford_tidybot(const std::string& urdf_file) {
    return Model<double>(urdf_file);
}

inline Model<double> model_from_name(const std::string& robot_name, const std::string& urdf_file = "",
                                     const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    if (robot_name == "ur") {
        return ur(g);
    }
    if (robot_name == "jaka") {
        return jaka(g);
    }
    if (robot_name == "franka") {
        return franka(g);
    }
    if (robot_name == "leap_left") {
        if (urdf_file.empty()) {
            throw std::invalid_argument("leap_left model requires a URDF file path");
        }
        return leap_hand(urdf_file);
    }
    if (robot_name == "stanford_tidybot" || robot_name == "tidybot_gen3_10dof") {
        if (urdf_file.empty()) {
            throw std::invalid_argument(robot_name + " model requires a URDF file path");
        }
        return stanford_tidybot(urdf_file);
    }
    throw std::invalid_argument("Unsupported robot model: " + robot_name);
}

struct TreeTemplateParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int dof = 0;
    int bf = 1;
    double taper = 1.0;
    double skew = 0.0;
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
    std::vector<int> parent_indices;
    std::vector<double> link_lengths;
    std::vector<double> masses;
    std::vector<Eigen::Matrix<double, 3, 1>> coms;
    std::vector<Eigen::Matrix<double, 6, 1>> inertia_tensors;
};

inline TreeTemplateParams make_tree_template_params(const int n, const int bf = 1,
                                                    const double taper = 1.0,
                                                    const double skew = 0.0,
                                                    const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    if (n < 0) {
        throw std::invalid_argument("make_tree_template_params: n must be non-negative");
    }
    if (bf < 1) {
        throw std::invalid_argument("make_tree_template_params: branching factor must be positive");
    }

    TreeTemplateParams params;
    params.dof = n;
    params.bf = bf;
    params.taper = taper;
    params.skew = skew;
    params.gravity = g;
    params.parent_indices.resize(static_cast<std::size_t>(n + 1), 0);
    params.link_lengths.resize(static_cast<std::size_t>(n), 0.0);
    params.masses.resize(static_cast<std::size_t>(n), 0.0);
    params.coms.resize(static_cast<std::size_t>(n), Eigen::Vector3d::Zero());
    params.inertia_tensors.resize(static_cast<std::size_t>(n), Eigen::Matrix<double, 6, 1>::Zero());

    for (int current = 1; current <= n; ++current) {
        params.parent_indices[static_cast<std::size_t>(current)] =
            current == 1 ? 0 : (current - 2) / bf + 1;

        const int seq = current - 1;
        const double Li = std::pow(taper, seq);
        const double mi = std::pow(taper, 3.0 * seq);
        const double Ixx = mi * Li * Li * 0.0025;
        const double Iyy = mi * Li * Li * (1.015 / 12.0);
        const double Izz = mi * Li * Li * (1.015 / 12.0);

        params.link_lengths[static_cast<std::size_t>(seq)] = Li;
        params.masses[static_cast<std::size_t>(seq)] = mi;
        params.coms[static_cast<std::size_t>(seq)] = Eigen::Vector3d(0.5 * Li, 0.0, 0.0);
        params.inertia_tensors[static_cast<std::size_t>(seq)] << Ixx, 0.0, 0.0, Iyy, 0.0, Izz;
    }
    return params;
}

inline Model<double> tree_model(const TreeTemplateParams& params, const std::string& model_name) {
    const int n = params.dof;
    const int num_bodies = n + 1;

    std::vector<char> joint_types(num_bodies, 'R');
    joint_types[0] = '\0';

    Eigen::Matrix<double, Eigen::Dynamic, 1> init_q(n);
    init_q.setZero();

    std::vector<Motor3D<double>> Mj(num_bodies);
    Mj[0] = ga_DH2motor<double>(Eigen::Vector4d(0, 0, 0, 0), true);
    for (int i = 0; i < n; ++i) {
        const int parent_body = params.parent_indices[static_cast<std::size_t>(i + 1)];
        const double parent_length =
            parent_body == 0 ? 0.0 : params.link_lengths[static_cast<std::size_t>(parent_body - 1)];
        Mj[i + 1] = ga_DH2motor<double>(Eigen::Vector4d(parent_length, params.skew, 0.0, 0.0), true);
    }
    std::vector<Line3D<double>> Lj(num_bodies, Line3D<double>(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

    Line3D<double> gravity = Line3D<double>::Zero();
    gravity.segment<3>(3) = params.gravity;

    std::vector<double> mass(num_bodies, 0.0);
    std::vector<Eigen::Matrix<double, 6, 1>> inertia_tensors(num_bodies, Eigen::Matrix<double, 6, 1>::Zero());
    std::vector<Eigen::Matrix<double, 3, 1>> CoMs(num_bodies, Eigen::Matrix<double, 3, 1>::Zero());
    for (int i = 0; i < n; ++i) {
        mass[i + 1] = params.masses[static_cast<std::size_t>(i)];
        CoMs[i + 1] = params.coms[static_cast<std::size_t>(i)];
        inertia_tensors[i + 1] = params.inertia_tensors[static_cast<std::size_t>(i)];
    }

    Eigen::Matrix<double, Eigen::Dynamic, 1> lower_position_limit(n);
    lower_position_limit.setConstant(-2.0 * M_PI);
    Eigen::Matrix<double, Eigen::Dynamic, 1> upper_position_limit(n);
    upper_position_limit.setConstant(2.0 * M_PI);
    Eigen::Matrix<double, Eigen::Dynamic, 1> velocity_limit(n);
    velocity_limit.setConstant(M_PI);
    Eigen::Matrix<double, Eigen::Dynamic, 1> effort_limit(n);
    effort_limit.setConstant(100.0);

    std::vector<SSL<double>> collision_ssl;

    return Model<double>(model_name, num_bodies, joint_types, params.parent_indices, init_q, Mj, Lj, gravity, mass,
                         inertia_tensors, CoMs, lower_position_limit, upper_position_limit, velocity_limit,
                         effort_limit, collision_ssl);
}

/****** serial chain n-dof ******/
inline Model<double> serial_chain(const int n, const double taper = 1.0, const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    return tree_model(make_tree_template_params(n, 1, taper, 0.0, g),
                      "serial_chain");
}

/****** binary tree n-dof ******/
inline Model<double> binary_tree(const int n, const double taper = 1.0, const Eigen::Vector3d& g = Eigen::Vector3d(0, 0, -9.81)) {
    return tree_model(make_tree_template_params(n, 2, taper, 0.0, g),
                      "binary_tree");
}

}  // namespace TetraPGA
