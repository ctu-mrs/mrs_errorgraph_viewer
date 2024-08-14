#pragma once
#include <cstdint>
#include "mrs_errorgraph_viewer/UavState.h"

// macro variables for the enum definition
#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME       uav_state_t
#define X_ENUM_BASE_TYPE  uint8_t
#define X_ENUM_SEQ                      \
                          (DISARMED)    \
                          (ARMED)       \
                          (OFFBOARD)    \
                          (MANUAL)      \
                          (TAKEOFF)     \
                          (LAND)        \
                          (RC_MODE)     \
                          (HOVER)       \
                          (GOTO)        \
                          (TRAJECTORY)

// optional macro variables for enum to ROS message conversions
#undef X_ENUM_MSG_TYPE
#undef X_ENUM_MSG_MEMBER
#undef X_ENUM_MSG_PREFIX

#define X_ENUM_MSG_TYPE mrs_errorgraph_viewer::UavState
#define X_ENUM_MSG_MEMBER state
#define X_ENUM_MSG_PREFIX STATE_

namespace mrs_errorgraph_viewer
{

#include "mrs_errorgraph_viewer/enums/enum_macros.h"

  // generate the enum and the to_string() conversion
DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

  // generate the to_ros() conversion
DEFINE_ENUM_MSG_CONVERSIONS(X_ENUM_NAME, X_ENUM_MSG_TYPE, X_ENUM_MSG_MEMBER, X_ENUM_MSG_PREFIX, X_ENUM_SEQ)

  // some more helper functions related to this enum
  inline bool is_flying(uav_state_t uav_state)
  {
    switch (uav_state)
    {
      case uav_state_t::DISARMED:
      case uav_state_t::ARMED:
      case uav_state_t::OFFBOARD:
        return false;
      default:
        return true;
    }
  }

}
