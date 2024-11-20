#ifndef POSE_PID__VISIBILITY_CONTROL_H_
#define POSE_PID__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POSE_PID_EXPORT __attribute__ ((dllexport))
    #define POSE_PID_IMPORT __attribute__ ((dllimport))
  #else
    #define POSE_PID_EXPORT __declspec(dllexport)
    #define POSE_PID_IMPORT __declspec(dllimport)
  #endif
  #ifdef POSE_PID_BUILDING_LIBRARY
    #define POSE_PID_PUBLIC POSE_PID_EXPORT
  #else
    #define POSE_PID_PUBLIC POSE_PID_IMPORT
  #endif
  #define POSE_PID_PUBLIC_TYPE POSE_PID_PUBLIC
  #define POSE_PID_LOCAL
#else
  #define POSE_PID_EXPORT __attribute__ ((visibility("default")))
  #define POSE_PID_IMPORT
  #if __GNUC__ >= 4
    #define POSE_PID_PUBLIC __attribute__ ((visibility("default")))
    #define POSE_PID_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POSE_PID_PUBLIC
    #define POSE_PID_LOCAL
  #endif
  #define POSE_PID_PUBLIC_TYPE
#endif

#endif  // POSE_PID__VISIBILITY_CONTROL_H_
