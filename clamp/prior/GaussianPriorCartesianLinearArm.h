/**
 *  @file   GaussianPriorCartesianLinearArm.h
 *  @brief  Gaussian prior factor for Arm's end effector state in Cartesian space 
 *  @author Mustafa Mukadam
 *  @date   April 17, 2017
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>

#include <clamp/prior/GaussianPriorCartesianLinear.h>

namespace clamp {

// template use ArmModel as robot type
typedef GaussianPriorCartesianLinear<gpmp2::ArmModel> GaussianPriorCartesianLinearArm;

}
