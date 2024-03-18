#ifndef MOTION_MANAGER_CPP__VISIBILITY_CONTROL_H_
#define MOTION_MANAGER_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_MANAGER_CPP_EXPORT __attribute__ ((dllexport))
    #define MOTION_MANAGER_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_MANAGER_CPP_EXPORT __declspec(dllexport)
    #define MOTION_MANAGER_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_MANAGER_CPP_BUILDING_DLL
    #define MOTION_MANAGER_CPP_PUBLIC MOTION_MANAGER_CPP_EXPORT
  #else
    #define MOTION_MANAGER_CPP_PUBLIC MOTION_MANAGER_CPP_IMPORT
  #endif
  #define MOTION_MANAGER_CPP_PUBLIC_TYPE MOTION_MANAGER_CPP_PUBLIC
  #define MOTION_MANAGER_CPP_LOCAL
#else
  #define MOTION_MANAGER_CPP_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_MANAGER_CPP_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_MANAGER_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_MANAGER_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_MANAGER_CPP_PUBLIC
    #define MOTION_MANAGER_CPP_LOCAL
  #endif
  #define MOTION_MANAGER_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MOTION_MANAGER_CPP__VISIBILITY_CONTROL_H_