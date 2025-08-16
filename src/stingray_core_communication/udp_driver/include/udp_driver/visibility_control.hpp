#ifndef UDP_DRIVER__VISIBILITY_CONTROL_HPP_
#define UDP_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(UDP_DRIVER_BUILDING_DLL) || defined(UDP_DRIVER_EXPORTS)
#define UDP_DRIVER_PUBLIC __declspec(dllexport)
#define UDP_DRIVER_LOCAL
#else  // defined(UDP_DRIVER_BUILDING_DLL) || defined(UDP_DRIVER_EXPORTS)
#define UDP_DRIVER_PUBLIC __declspec(dllimport)
#define UDP_DRIVER_LOCAL
#endif  // defined(UDP_DRIVER_BUILDING_DLL) || defined(UDP_DRIVER_EXPORTS)
#elif defined(__linux__)
#define UDP_DRIVER_PUBLIC __attribute__((visibility("default")))
#define UDP_DRIVER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define UDP_DRIVER_PUBLIC __attribute__((visibility("default")))
#define UDP_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // UDP_DRIVER__VISIBILITY_CONTROL_HPP_
