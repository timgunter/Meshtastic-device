#pragma once

#include "DebugConfiguration.h"

/// Macros to instrument all calls to the logger with a user defined prefix. Allows
/// one to add a prefix to all logger calls in a given translation unit.

/// Note: This assumes the first __VA_ARGS__ is a char * and can be concatenated with LOG_PREFIX
#define LOG_DEBUG_PFX(...) LOG_DEBUG(LOG_PREFIX __VA_ARGS__)
#define LOG_INFO_PFX( ...) LOG_INFO( LOG_PREFIX __VA_ARGS__)
#define LOG_WARN_PFX( ...) LOG_WARN( LOG_PREFIX __VA_ARGS__)
#define LOG_ERROR_PFX(...) LOG_ERROR(LOG_PREFIX __VA_ARGS__)
#define LOG_CRIT_PFX( ...) LOG_CRIT( LOG_PREFIX __VA_ARGS__)
#define LOG_TRACE_PFX(...) LOG_TRACE(LOG_PREFIX __VA_ARGS__)

/// #defines to promote debug and info messages to make them stand out
#ifdef LOG_DEBUG_AS_INFO
#   undef  LOG_DEBUG_PFX
#   define LOG_DEBUG_PFX(...) LOG_INFO(LOG_PREFIX __VA_ARGS__)
#endif

#ifdef LOG_DEBUG_AS_WARN
#   undef  LOG_DEBUG_PFX
#   define LOG_DEBUG_PFX(...) LOG_WARN(LOG_PREFIX __VA_ARGS__)
#endif

#ifdef LOG_INFO_AS_WARN
#   undef  LOG_INFO_PFX
#   define LOG_INFO_PFX(...) LOG_WARN(LOG_PREFIX __VA_ARGS__)
#endif
