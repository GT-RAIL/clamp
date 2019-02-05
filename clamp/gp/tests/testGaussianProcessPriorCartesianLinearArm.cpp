/**
*  @file    testGaussianProcessPriorCartesianLinearArm.cpp
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

#include <clamp/gp/GaussianProcessPriorCartesianLinearArm.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;
using namespace clamp;

/* ************************************************************************** */
TEST_UNSAFE(GaussianProcessPriorCartesianLinearArm, Factor) { 
  
  Vector6 p1, p2, v1, v2;
  Matrix actualH1, actualH2, actualH3, actualH4;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Vector actual, expect;

  Vector Nu(6);
  Nu << 5.846e-06,  4.896e-06, -1.971e-07,  5.552e-05,  5.065e-05, -1.927e-06;
  Matrix Phi(6,6), Q(6,6);
  Phi <<
   1.000e+00,  0.000e+00,  0.000e+00,  3.999e-01,  0.000e+00,  0.000e+00,
   0.000e+00,  1.000e+00,  0.000e+00,  0.000e+00,  4.000e-01,  0.000e+00,
   0.000e+00,  0.000e+00,  1.000e+00,  0.000e+00,  0.000e+00,  4.001e-01,
  -2.766e-04,  0.000e+00,  0.000e+00,  9.987e-01,  0.000e+00,  0.000e+00,
   0.000e+00,  5.733e-04,  0.000e+00,  0.000e+00,  9.995e-01,  0.000e+00,
   0.000e+00,  0.000e+00,  1.651e-05,  0.000e+00,  0.000e+00,  1.001e+00;
  Q <<
   1.635e-30,  0.000e+00,  0.000e+00, -6.115e-32,  0.000e+00,  0.000e+00,
   0.000e+00,  1.526e-30,  0.000e+00,  0.000e+00,  8.272e-31,  0.000e+00,
   0.000e+00,  0.000e+00,  1.348e-33,  0.000e+00,  0.000e+00, -2.915e-35,
  -6.115e-32,  0.000e+00,  0.000e+00,  4.634e-33,  0.000e+00,  0.000e+00,
   0.000e+00,  8.272e-31,  0.000e+00,  0.000e+00,  6.691e-31,  0.000e+00,
   0.000e+00,  0.000e+00, -2.915e-35,  0.000e+00,  0.000e+00,  2.043e-36;

  BodySphereVector spheres;
  spheres.push_back(BodySphere(5, 0.01, Point3(0, 0, 0)));
  ArmModel end_eff;
  Vector a(6), alpha(6), d(6), theta(6);
  alpha << 1.5708, 3.1416, 1.5708, 1.0472, 1.0472, 3.1416;
  a << 0, 0.41, 0, 0, 0, 0;
  d << 0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228;
  theta << 0, -1.5708, 1.5708, 0, -3.1416, 1.5708;
  end_eff = ArmModel(Arm(6, a, alpha, d, Pose3(Rot3(Matrix::Identity(3,3)), Point3(0,0,0)), theta), spheres);

  GaussianProcessPriorCartesianLinearArm factor(Symbol('x', 0), Symbol('v', 0), 
    Symbol('x', 1), Symbol('v', 1), end_eff, Phi, Nu, Q);

  // test at some configuration with no velocity
  p1 = (Vector(6) << -8.991e+00,  4.479e+00,  1.166e+00,  6.747e+00,  1.881e+00,  2.453e+00).finished();
  v1 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  p2 = (Vector(6) << -9.017e+00,  4.488e+00,  1.151e+00,  6.737e+00,  1.879e+00,  2.453e+00).finished();
  v2 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(6) << 1.103e-02,  1.154e-02,  2.341e-04, -4.255e-05, -1.401e-05, -4.434e-07).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-3));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // test at same configuration with no velocity
  p1 = (Vector(6) << 0, 3.14, 3.14, 0, 0, 0).finished();
  v1 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  p2 = (Vector(6) << 0, 3.14, 3.14, 0, 0, 0).finished();
  v2 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(6) << 5.827e-06, 8.447e-06, 1.835e-06, 5.534e-05, 8.751e-05, 1.791e-05).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorCartesianLinearArm::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-3));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
