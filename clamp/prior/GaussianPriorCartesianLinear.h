/**
 *  @file   GaussianPriorCartesianLinear.h
 *  @brief  Gaussian prior defined on any state of trajectory in Cartesian workspace
 *  @author Mustafa Mukadam
 *  @date   April 17, 2017
 **/


#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <gpmp2/kinematics/RobotModel.h>

namespace clamp {

/**
 * 2-way factor for Gaussian Cartesian prior, linear version
 */
template <class ROBOT>
class GaussianPriorCartesianLinear: public gtsam::NoiseModelFactor2<
    typename ROBOT::Pose, typename ROBOT::Velocity> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;
  typedef typename Robot::Velocity Velocity;

private:
  // typedefs
  typedef GaussianPriorCartesianLinear This;
  typedef gtsam::NoiseModelFactor2<Pose, Velocity> Base;

  // Robot with sphere information
  const Robot& end_eff_;

  gtsam::Vector Mu_;
  size_t dof_;


public:

  /* Default constructor do nothing */
  GaussianPriorCartesianLinear() : end_eff_(Robot()) {}

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianPriorCartesianLinear(gtsam::Key poseKey1, gtsam::Key velKey1, const Robot& end_eff, 
      const gtsam::Vector& Mu, const gtsam::Matrix& Sigma) :
        
        Base(gtsam::noiseModel::Gaussian::Covariance(Sigma), poseKey1, velKey1), 
        end_eff_(end_eff), Mu_(Mu), dof_(Mu.size()/2) {}

  virtual ~GaussianPriorCartesianLinear() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


  /// factor error function
  gtsam::Vector evaluateError(
      const Pose& p, const Velocity& v,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    std::vector<Point3> px;
    std::vector<Vector3> vx;
    std::vector<Matrix> J_px_p, J_vx_v;

    if (H1 || H2)
      end_eff_.sphereCentersAndVels(p, v, px, vx, J_px_p, J_vx_v);
    else
      end_eff_.sphereCentersAndVels(p, v, px, vx);

    // Cartesian state vector
    Vector x = (Vector(6) << px[0].vector(), vx[0]).finished();

    // Jacobians
    if (H1) *H1 = (Matrix(2*dof_, dof_) << Matrix::Identity(dof_, dof_), 
      Matrix::Zero(dof_, dof_)).finished() * J_px_p[0];
    if (H2) *H2 = (Matrix(2*dof_, dof_) << Matrix::Zero(dof_, dof_), 
      Matrix::Identity(dof_, dof_)).finished() * J_vx_v[0];

    // error
    return x - Mu_;
  }


  /** dimensions */
  size_t dim() const { return dof_; }

  /** number of variables attached to this factor */
  size_t size() const {
    return 2;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && (this->Mu_ - e->Mu_).norm() < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "2-way Gaussian Cartesian Factor Linear(" << dof_ << ")" << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(dof_);
    ar & BOOST_SERIALIZATION_NVP(Mu_); 
  }

}; // GaussianPriorCartesianLinear

} // namespace clamp
