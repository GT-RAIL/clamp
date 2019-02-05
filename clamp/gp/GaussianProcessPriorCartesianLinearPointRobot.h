/**
 *  @file   GaussianPriorCartesianLinearArm.h
 *  @brief  Gaussian prior factor for PointRobot's state in Cartesian space 
 *  @author Mustafa Mukadam, Asif Rana
 *  @date   Februrary 17, 2018
 **/

#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>

#include <clamp/gp/GaussianProcessPriorCartesianLinear.h>

namespace clamp {

// template use ArmModel as robot type
typedef GaussianProcessPriorCartesianLinear<gpmp2::PointRobotModel> GaussianProcessPriorCartesianLinearPointRobot;

}

