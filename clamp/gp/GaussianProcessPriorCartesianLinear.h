/**
 *  @file   GaussianProcessPriorCartesianLinear.h
 *  @brief  GP prior defined on end effector trajectory in Cartesian workspace
 *  @author Mustafa Mukadam
 *  @date   April 7, 2017
 **/


#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <gpmp2/kinematics/RobotModel.h>

namespace clamp {

/**
 * 4-way factor for Gaussian Process Cartesian prior, linear version
 */
template <class ROBOT>
class GaussianProcessPriorCartesianLinear: public gtsam::NoiseModelFactor4<
    typename ROBOT::Pose, typename ROBOT::Velocity, 
    typename ROBOT::Pose, typename ROBOT::Velocity> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;
  typedef typename Robot::Velocity Velocity;

private:
  // typedefs
  typedef GaussianProcessPriorCartesianLinear This;
  typedef gtsam::NoiseModelFactor4<Pose, Velocity, Pose, Velocity> Base;

  // Robot with sphere information
  const Robot& end_eff_;

  gtsam::Matrix Phi_;
  gtsam::Vector Nu_;
  size_t dof_;


public:

  /* Default constructor do nothing */
  GaussianProcessPriorCartesianLinear() : end_eff_(Robot()) {}

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorCartesianLinear(gtsam::Key poseKey1, gtsam::Key velKey1, 
      gtsam::Key poseKey2, gtsam::Key velKey2, const Robot& end_eff, 
      const gtsam::Matrix& Phi, const gtsam::Vector& Nu, const gtsam::Matrix& Q) :
        
        Base(gtsam::noiseModel::Gaussian::Covariance(Q), poseKey1, velKey1, poseKey2, velKey2), 
        end_eff_(end_eff), Phi_(Phi), Nu_(Nu), dof_(Nu.size()/2) {}

  virtual ~GaussianProcessPriorCartesianLinear() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


  /// factor error function
  gtsam::Vector evaluateError(
      const Pose& p1, const Velocity& v1, const Pose& p2, const Velocity& v2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    using namespace gtsam;

    std::vector<Point3> px1, px2;
    std::vector<Vector3> vx1, vx2;
    std::vector<Matrix> J_px1_p1, J_vx1_v1, J_px2_p2, J_vx2_v2;

    if (H1 || H2)
      end_eff_.sphereCentersAndVels(p1, v1, px1, vx1, J_px1_p1, J_vx1_v1);
    else
      end_eff_.sphereCentersAndVels(p1, v1, px1, vx1);
    if (H3 || H4)
      end_eff_.sphereCentersAndVels(p2, v2, px2, vx2, J_px2_p2, J_vx2_v2);
    else
      end_eff_.sphereCentersAndVels(p2, v2, px2, vx2);

    // Cartesian state vector
    Vector x1 = (Vector(6) << px1[0].vector(), vx1[0]).finished();
    Vector x2 = (Vector(6) << px2[0].vector(), vx2[0]).finished();

    // Jacobians
    if (H1) *H1 = (Matrix(2*dof_, dof_) << Phi_.block(0, 0, 2*dof_, dof_)).finished() * J_px1_p1[0];
    if (H2) *H2 = (Matrix(2*dof_, dof_) << Phi_.block(0, dof_, 2*dof_, dof_)).finished() * J_vx1_v1[0];
    if (H3) *H3 = (Matrix(2*dof_, dof_) << -1.0*Matrix::Identity(dof_, dof_), 
      Matrix::Zero(dof_, dof_)).finished() * J_px2_p2[0];
    if (H4) *H4 = (Matrix(2*dof_, dof_) << Matrix::Zero(dof_, dof_), 
      -1.0*Matrix::Identity(dof_, dof_)).finished() * J_vx2_v2[0];

    // transition matrix & error
    return Phi_*x1 - x2 + Nu_;
  }


  /** dimensions */
  size_t dim() const { return dof_; }

  /** number of variables attached to this factor */
  size_t size() const {
    return 4;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && (this->Nu_ - e->Nu_).norm() < tol
      && (this->Phi_ - e->Phi_).norm() < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "4-way Gaussian Process Cartesian Factor Linear(" << dof_ << ")" << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(dof_);
    ar & BOOST_SERIALIZATION_NVP(Phi_);
    ar & BOOST_SERIALIZATION_NVP(Nu_); 
  }

}; // GaussianProcessPriorCartesianLinear

} // namespace clamp
