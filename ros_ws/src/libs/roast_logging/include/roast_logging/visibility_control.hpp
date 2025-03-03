
#ifndef ROAST_LOGGING__VISIBILITY_CONTROL_H_
#define ROAST_LOGGING__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROAST_LOGGING_EXPORT __attribute__((dllexport))
#define ROAST_LOGGING_IMPORT __attribute__((dllimport))
#else
#define ROAST_LOGGING_EXPORT __declspec(dllexport)
#define ROAST_LOGGING_IMPORT __declspec(dllimport)
#endif
#ifdef ROAST_LOGGING_BUILDING_DLL
#define ROAST_LOGGING_PUBLIC ROAST_LOGGING_EXPORT
#else
#define ROAST_LOGGING_PUBLIC ROAST_LOGGING_IMPORT
#endif
#define ROAST_LOGGING_PUBLIC_TYPE ROAST_LOGGING_PUBLIC
#define ROAST_LOGGING_LOCAL
#else
#define ROAST_LOGGING_EXPORT __attribute__((visibility("default")))
#define ROAST_LOGGING_IMPORT
#if __GNUC__ >= 4
#define ROAST_LOGGING_PUBLIC __attribute__((visibility("default")))
#define ROAST_LOGGING_LOCAL __attribute__((visibility("hidden")))
#else
#define ROAST_LOGGING_PUBLIC
#define ROAST_LOGGING_LOCAL
#endif
#define ROAST_LOGGING_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROAST_LOGGING__VISIBILITY_CONTROL_H_
