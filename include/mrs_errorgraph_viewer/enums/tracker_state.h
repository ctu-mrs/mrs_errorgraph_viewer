#pragma once
#include <cstdint>
#include "mrs_errorgraph_viewer/enums/enum_helpers.h"

#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME       tracker_state_t
#define X_ENUM_BASE_TYPE  uint8_t
#define X_ENUM_SEQ                            \
                          (NULL_TRACKER)      \
                          (TAKEOFF)           \
                          (HOVER)             \
                          (REFERENCE)         \
                          (TRAJECTORY)        \
                          (LAND)           

namespace mrs_errorgraph_viewer
{

DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

}
