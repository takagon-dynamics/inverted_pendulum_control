#ifndef INVERTED_PENDULUM_CONTROL__VISIBILITY_CONTROL_H_
#define INVERTED_PENDULUM_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define INVERTED_PENDULUM_CONTROL_EXPORT __attribute__((dllexport))
#define INVERTED_PENDULUM_CONTROL_IMPORT __attribute__((dllimport))
#else
#define INVERTED_PENDULUM_CONTROL_EXPORT __declspec(dllexport)
#define INVERTED_PENDULUM_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef INVERTED_PENDULUM_CONTROL_BUILDING_DLL
#define INVERTED_PENDULUM_CONTROL_PUBLIC INVERTED_PENDULUM_CONTROL_EXPORT
#else
#define INVERTED_PENDULUM_CONTROL_PUBLIC INVERTED_PENDULUM_CONTROL_IMPORT
#endif
#define INVERTED_PENDULUM_CONTROL_PUBLIC_TYPE INVERTED_PENDULUM_CONTROL_PUBLIC
#define INVERTED_PENDULUM_CONTROL_LOCAL
#else
#define INVERTED_PENDULUM_CONTROL_EXPORT __attribute__((visibility("default")))
#define INVERTED_PENDULUM_CONTROL_IMPORT
#if __GNUC__ >= 4
#define INVERTED_PENDULUM_CONTROL_PUBLIC __attribute__((visibility("default")))
#define INVERTED_PENDULUM_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define INVERTED_PENDULUM_CONTROL_PUBLIC
#define INVERTED_PENDULUM_CONTROL_LOCAL
#endif
#define INVERTED_PENDULUM_CONTROL_PUBLIC_TYPE
#endif

#endif  // INVERTED_PENDULUM_CONTROL__VISIBILITY_CONTROL_H_
