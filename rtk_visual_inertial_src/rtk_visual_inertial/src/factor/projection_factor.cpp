

#include "projection_factor.h"
#include "../utility/utility.h"
#include "../parameter/parameters.h"


projection_factor::projection_factor(const Eigen::Vector3d& _pts) : pts(_pts) {

};

bool projection_factor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d tic2(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic2(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d FeaturePos(parameters[2][0], parameters[2][1], parameters[2][2]);

    Eigen::Vector3d pts_imu_j = Qj.inverse() * (FeaturePos - Pj);
    Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j + Pbg - tic2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts.head<2>();
    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric2.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric2.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose1(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = - ric2.transpose();
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose1.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric2.transpose() * Rj.transpose();
        }
    }

    return true;
}





ProjectionTwoFrameTwoCamFactor::ProjectionTwoFrameTwoCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j) :
    pts_i(_pts_i), pts_j(_pts_j) {

};

bool ProjectionTwoFrameTwoCamFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d tic2(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qic2(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    double inv_dep_i = parameters[4][0];




    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic - Pbg;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (Qi * pts_imu_i + (Pi - Pj));
    Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j + Pbg - tic2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric2.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric2.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric2.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose() * Rj.transpose() * Ri;
            jaco_ex.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri * ric * -Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose1(jacobians[3]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = - ric2.transpose();
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose1.rightCols<1>().setZero();
        }
        if (jacobians[4]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[4]);
            jacobian_feature = reduce * ric2.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
        }

    }


    return true;
}



ProjectionTwoFrameOneCamFactor::ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j) :
    pts_i(_pts_i), pts_j(_pts_j) {



};

bool ProjectionTwoFrameOneCamFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double inv_dep_i = parameters[3][0];


    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic - Pbg;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (Qi * pts_imu_i + (Pi - Pj));
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j + Pbg - tic);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                     Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * ( tic - Pbg) + (Pi - Pj)) + Pbg - tic));
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
        }

    }


    return true;
}




ProjectionOneFrameTwoCamFactor::ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j ):
    pts_i(_pts_i), pts_j(_pts_j) {


};

bool ProjectionOneFrameTwoCamFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d tic(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qic(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tic2(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic2(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_dep_i = parameters[2][0];

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_imu_j = pts_imu_i;
    Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j - tic2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[0]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose();
            jaco_ex.rightCols<3>() = ric2.transpose() * ric * -Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose1(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = - ric2.transpose();
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose1.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric2.transpose() * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);

        }

    }


    return true;
}






































projection_factorNoEx1::projection_factorNoEx1(const Eigen::Vector3d& _pts) :
    pts(_pts) {

};

bool projection_factorNoEx1::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d FeaturePos(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj_inverse = Qj.inverse();
    Eigen::Vector3d pts_imu_j = Qj_inverse * (FeaturePos - Pj);
    Eigen::Vector3d pts_camera_j = QCI[0] * (pts_imu_j + Pcg[0]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts.head<2>();
    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Rj_transpose = Qj_inverse.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = RCI[0] * -Rj_transpose;
            jaco_j.rightCols<3>() = RCI[0] * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_feature(jacobians[1]);
            jacobian_feature = reduce * RCI[0] * Rj_transpose;
        }
    }

    return true;
}



projection_factorNoEx2::projection_factorNoEx2(const Eigen::Vector3d& _pts) :
    pts(_pts) {

};

bool projection_factorNoEx2::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d FeaturePos(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj_inverse = Qj.inverse();
    Eigen::Vector3d pts_imu_j = Qj_inverse * (FeaturePos - Pj);
    Eigen::Vector3d pts_camera_j = QCI[1] * (pts_imu_j + Pcg[1]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts.head<2>();
    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Rj_transpose = Qj_inverse.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = RCI[1] * -Rj_transpose;
            jaco_j.rightCols<3>() = RCI[1] * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_feature(jacobians[1]);
            jacobian_feature = reduce * RCI[1] * Rj_transpose;
        }
    }

    return true;
}




ProjectionTwoFrameTwoCamFactorNoEx::ProjectionTwoFrameTwoCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j,
        const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_, const Eigen::Vector3d& tic2_, const Eigen::Quaterniond& qic2_) :
    pts_i(_pts_i), pts_j(_pts_j), tic(tic_), qic(qic_), tic2(tic2_), qic2(qic2_) {

};

bool ProjectionTwoFrameTwoCamFactorNoEx::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);


    double inv_dep_i = parameters[2][0];




    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic - Pbg;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (Qi * pts_imu_i + (Pi - Pj));
    Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j + Pbg - tic2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric2.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric2.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric2.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric2.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
        }

    }


    return true;
}



ProjectionTwoFrameOneCamFactorNoEx::ProjectionTwoFrameOneCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j,
        const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_) :
    pts_i(_pts_i), pts_j(_pts_j), tic(tic_), qic(qic_) {



};

bool ProjectionTwoFrameOneCamFactorNoEx::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);


    double inv_dep_i = parameters[2][0];


    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic - Pbg;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (Qi * pts_imu_i + (Pi - Pj));
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j + Pbg - tic);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
        }

    }


    return true;
}





ProjectionOneFrameTwoCamFactorNoEx::ProjectionOneFrameTwoCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j,
        const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_, const Eigen::Vector3d& tic2_, const Eigen::Quaterniond& qic2_ ):
    pts_i(_pts_i), pts_j(_pts_j), tic(tic_), qic(qic_), tic2(tic2_), qic2(qic2_) {


};

bool ProjectionOneFrameTwoCamFactorNoEx::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

    if (parameters == 0) {
        residuals[0] = 0;
        return false;
    };
    double inv_dep_i = parameters[0][0];

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_imu_j = pts_imu_i;
    Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j - tic2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();


    residual = SQRT_INFO_FEATURE * residual;

    if (jacobians) {
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
               0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = SQRT_INFO_FEATURE * reduce;


        if (jacobians[0]) {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[0]);
            jacobian_feature = reduce * ric2.transpose() * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);

        }

    }


    return true;
}

