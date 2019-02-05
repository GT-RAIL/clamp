/**
 *  @file   GaussianProcessPriorCartesianLinearArm.h
 *  @brief  GP prior factor for Arm's end effector in Cartesian space 
 *  @author Mustafa Mukadam
 *  @date   April 10, 2017
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>

#include <clamp/gp/GaussianProcessPriorCartesianLinear.h>

namespace clamp {

// template use ArmModel as robot type
typedef GaussianProcessPriorCartesianLinear<gpmp2::ArmModel> GaussianProcessPriorCartesianLinearArm;

}
