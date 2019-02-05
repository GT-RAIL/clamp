/**
 *  @file   GaussianPriorCartesianLinearPointRobot.h
 *  @brief  Gaussian prior factor for PointRobots's state in Cartesian space 
 *  @author Mustafa Mukadam, Asif Rana
 *  @date   February 17, 2018
 **/

#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>

#include <clamp/prior/GaussianPriorCartesianLinear.h>

namespace clamp {

// template use ArmModel as robot type
typedef GaussianPriorCartesianLinear<gpmp2::PointRobotModel> GaussianPriorCartesianLinearPointRobot;

}
