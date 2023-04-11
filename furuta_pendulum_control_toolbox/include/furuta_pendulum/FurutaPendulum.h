#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
// define namespace and base
#define ROBCOGEN_NS FurutaPendulum
#define TARGET_NS FurutaPendulum
// define the links
#define CT_BASE fr_base_link
#define CT_L0 fr_arm1
#define CT_L1 fr_arm2
// define single end effector (could also be multiple)
#define CT_N_EE 1
// #define CT_EE0 fr_ee
// #define CT_EE0_IS_ON_LINK 2
// #define CT_EE0_FIRST_JOINT 0
// #define CT_EE0_LAST_JOINT 1
#include <ct/rbd/robot/robcogen/robcogenHelpers.h>