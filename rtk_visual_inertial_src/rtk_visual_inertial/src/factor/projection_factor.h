

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>



//feature poistion factor
class projection_factor : public ceres::SizedCostFunction<2, 7, 7, 3> {
  public:
    projection_factor( const Eigen::Vector3d& _pts);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts;

};

//feature inverse depth factor
class ProjectionOneFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 1> {
  public:
    ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts_i, pts_j;

};


//feature inverse depth factor
class ProjectionTwoFrameOneCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
  public:
    ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts_i, pts_j;

};


//feature inverse depth factor
class ProjectionTwoFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 1> {
  public:
    ProjectionTwoFrameTwoCamFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts_i, pts_j;

};







//feature poistion factor
class projection_factorNoEx1 : public ceres::SizedCostFunction<2, 7, 3> {
  public:
    projection_factorNoEx1( const Eigen::Vector3d& _pts);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts;

};
//feature poistion factor
class projection_factorNoEx2 : public ceres::SizedCostFunction<2, 7, 3> {
  public:
    projection_factorNoEx2( const Eigen::Vector3d& _pts);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d pts;

};

//feature inverse depth factor
class ProjectionOneFrameTwoCamFactorNoEx : public ceres::SizedCostFunction<2, 1> {
  public:
    ProjectionOneFrameTwoCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j
                                       , const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_, const Eigen::Vector3d& tic2_, const Eigen::Quaterniond& qic2_);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;


    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d tic;
    Eigen::Quaterniond qic;
    Eigen::Vector3d tic2;
    Eigen::Quaterniond qic2;

};


//feature inverse depth factor
class ProjectionTwoFrameOneCamFactorNoEx : public ceres::SizedCostFunction<2, 7, 7, 1> {
  public:
    ProjectionTwoFrameOneCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j
                                       , const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;



    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d tic;
    Eigen::Quaterniond qic;

};


//feature inverse depth factor
class ProjectionTwoFrameTwoCamFactorNoEx : public ceres::SizedCostFunction<2, 7, 7, 1> {
  public:
    ProjectionTwoFrameTwoCamFactorNoEx(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j,
                                       const Eigen::Vector3d& tic_, const Eigen::Quaterniond& qic_, const Eigen::Vector3d& tic2_, const Eigen::Quaterniond& qic2_);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;



    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d tic;
    Eigen::Quaterniond qic;

    Eigen::Vector3d tic2;
    Eigen::Quaterniond qic2;

};
