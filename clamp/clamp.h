// clamp matlab wrapper declarations

// gtsam declaration
virtual class gtsam::NoiseModelFactor;



namespace clamp {

////////////////////////////////////////////////////////////////////////////////
// gp
////////////////////////////////////////////////////////////////////////////////

#include <clamp/gp/GaussianProcessPriorCartesianLinearArm.h>

virtual class GaussianProcessPriorCartesianLinearArm : gtsam::NoiseModelFactor {
  GaussianProcessPriorCartesianLinearArm(size_t key1, size_t key2, size_t key3, size_t key4,
      const gpmp2::ArmModel& end_eff, Matrix Phi, Vector Nu, Matrix Q);
};

#include <clamp/gp/GaussianProcessPriorCartesianLinearPointRobot.h>

virtual class GaussianProcessPriorCartesianLinearPointRobot : gtsam::NoiseModelFactor {
  GaussianProcessPriorCartesianLinearPointRobot(size_t key1, size_t key2, size_t key3, size_t key4,
      const gpmp2::PointRobotModel& point_robot, Matrix Phi, Vector Nu, Matrix Q);
};



////////////////////////////////////////////////////////////////////////////////
// prior
////////////////////////////////////////////////////////////////////////////////

#include <clamp/prior/GaussianPriorCartesianLinearArm.h>

virtual class GaussianPriorCartesianLinearArm : gtsam::NoiseModelFactor {
  GaussianPriorCartesianLinearArm(size_t key1, size_t key2, 
      const gpmp2::ArmModel& end_eff, Vector Mu, Matrix Sigma);
};

#include <clamp/prior/GaussianPriorCartesianLinearPointRobot.h>

virtual class GaussianPriorCartesianLinearPointRobot : gtsam::NoiseModelFactor {
  GaussianPriorCartesianLinearPointRobot(size_t key1, size_t key2, 
      const gpmp2::PointRobotModel& point_robot, Vector Mu, Matrix Sigma);
};


} // namespace clamp
