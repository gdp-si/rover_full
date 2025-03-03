#ifndef ROAST_BEHAVIOR_TREE__VISIBILITY_CONTROL_H_
#define ROAST_BEHAVIOR_TREE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROAST_BEHAVIOR_TREE_EXPORT __attribute__((dllexport))
#define ROAST_BEHAVIOR_TREE_IMPORT __attribute__((dllimport))
#else
#define ROAST_BEHAVIOR_TREE_EXPORT __declspec(dllexport)
#define ROAST_BEHAVIOR_TREE_IMPORT __declspec(dllimport)
#endif
#ifdef ROAST_BEHAVIOR_TREE_BUILDING_LIBRARY
#define ROAST_BEHAVIOR_TREE_PUBLIC ROAST_BEHAVIOR_TREE_EXPORT
#else
#define ROAST_BEHAVIOR_TREE_PUBLIC ROAST_BEHAVIOR_TREE_IMPORT
#endif
#define ROAST_BEHAVIOR_TREE_PUBLIC_TYPE ROAST_BEHAVIOR_TREE_PUBLIC
#define ROAST_BEHAVIOR_TREE_LOCAL
#else
#define ROAST_BEHAVIOR_TREE_EXPORT __attribute__((visibility("default")))
#define ROAST_BEHAVIOR_TREE_IMPORT
#if __GNUC__ >= 4
#define ROAST_BEHAVIOR_TREE_PUBLIC __attribute__((visibility("default")))
#define ROAST_BEHAVIOR_TREE_LOCAL __attribute__((visibility("hidden")))
#else
#define ROAST_BEHAVIOR_TREE_PUBLIC
#define ROAST_BEHAVIOR_TREE_LOCAL
#endif
#define ROAST_BEHAVIOR_TREE_PUBLIC_TYPE
#endif

#endif  // ROAST_BEHAVIOR_TREE__VISIBILITY_CONTROL_H_
