#ifndef ACTION_TURTLE_COMMANDS__VISIBILITY_CONTROL_H_
#define ACTION_TURTLE_COMMANDS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TURTLE_COMMANDS_EXPORT __attribute__ ((dllexport))
    #define ACTION_TURTLE_COMMANDS_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TURTLE_COMMANDS_EXPORT __declspec(dllexport)
    #define ACTION_TURTLE_COMMANDS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TURTLE_COMMANDS_BUILDING_DLL
    #define ACTION_TURTLE_COMMANDS_PUBLIC ACTION_TURTLE_COMMANDS_EXPORT
  #else
    #define ACTION_TURTLE_COMMANDS_PUBLIC ACTION_TURTLE_COMMANDS_IMPORT
  #endif
  #define ACTION_TURTLE_COMMANDS_PUBLIC_TYPE ACTION_TURTLE_COMMANDS_PUBLIC
  #define ACTION_TURTLE_COMMANDS_LOCAL
#else
  #define ACTION_TURTLE_COMMANDS_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TURTLE_COMMANDS_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TURTLE_COMMANDS_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TURTLE_COMMANDS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TURTLE_COMMANDS_PUBLIC
    #define ACTION_TURTLE_COMMANDS_LOCAL
  #endif
  #define ACTION_TURTLE_COMMANDS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TURTLE_COMMANDS__VISIBILITY_CONTROL_H_