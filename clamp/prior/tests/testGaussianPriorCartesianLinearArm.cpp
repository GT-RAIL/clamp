/**
*  @file    testGaussianPriorCartesianLinearArm.cpp
*  @author  Mustafa Mukadam
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <clamp/prior/GaussianPriorCartesianLinearArm.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;
using namespace clamp;

/* ************************************************************************** */
TEST_UNSAFE(GaussianPriorCartesianLinearArm, Factor) { 
  
  Vector6 p, v;
  Matrix actualH1, actualH2;
  Matrix expectH1, expectH2;
  Vector actual, expect;

  Vector Mu(6);
  Mu << 0, 0, 0, 0, 0, 0;
  Matrix Sigma(6,6);
  Sigma << 1e-2*Matrix::Identity(6, 6);

  BodySphereVector spheres;
  spheres.push_back(BodySphere(5, 0.01, Point3(0, 0, 0)));
  ArmModel end_eff;
  Vector a(6), alpha(6), d(6), theta(6);
  alpha << 1.5708, 3.1416, 1.5708, 1.0472, 1.0472, 3.1416;
  a << 0, 0.41, 0, 0, 0, 0;
  d << 0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228;
  theta << 0, -1.5708, 1.5708, 0, -3.1416, 1.5708;
  end_eff = ArmModel(Arm(6, a, alpha, d, Pose3(Rot3(Matrix::Identity(3,3)), Point3(0,0,0)), theta), spheres);

  GaussianPriorCartesianLinearArm factor(Symbol('x', 0), Symbol('v', 0), end_eff, Mu, Sigma);

  // test at some configuration with no velocity
  p = (Vector(6) << -8.991e+00,  4.479e+00,  1.166e+00,  6.747e+00,  1.881e+00,  2.453e+00).finished();
  v = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p, v, actualH1, actualH2);
  expect = (Vector(6) << 0.3546, -0.1127, 0.0898, 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianPriorCartesianLinearArm::evaluateError, factor,
          _1, v, boost::none, boost::none)), p, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianPriorCartesianLinearArm::evaluateError, factor,
          p, _1, boost::none, boost::none)), v, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-4));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));

  // test at same configuration with no velocity
  p = (Vector(6) << 0, 3.14, 3.14, 0, 0, 0).finished();
  v = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p, v, actualH1, actualH2);
  expect = (Vector(6) << 0.0006, 0.0643, 1.2011, 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianPriorCartesianLinearArm::evaluateError, factor,
          _1, v, boost::none, boost::none)), p, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianPriorCartesianLinearArm::evaluateError, factor,
          p, _1, boost::none, boost::none)), v, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-4));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
