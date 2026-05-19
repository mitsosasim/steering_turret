/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

#pragma once

#include <string>
#include <string_view>

#ifndef CU_CORE_VERSION_MAJOR
#define CU_CORE_VERSION_MAJOR 0
#endif

#ifndef CU_CORE_VERSION_MINOR
#define CU_CORE_VERSION_MINOR 0
#endif

#ifndef CU_CORE_VERSION_PATCH
#define CU_CORE_VERSION_PATCH 0
#endif

#ifndef CU_CORE_VERSION_STRING
#define CU_CORE_VERSION_STRING "0.0.0"
#endif

#ifndef CU_CORE_GIT_COMMIT
#define CU_CORE_GIT_COMMIT "unknown"
#endif

// Header centralizing CU Core version info. Values come from the build system
// (CMake defines CU_CORE_VERSION_* macros for every target) and fallback to
// sensible defaults when the header is used outside the normal build.
// To modify version info, edit the CMakeLists.txt of the CU Core project.
namespace version_info {

constexpr int major() noexcept {
    return CU_CORE_VERSION_MAJOR;
}

constexpr int minor() noexcept {
    return CU_CORE_VERSION_MINOR;
}

constexpr int patch() noexcept {
    return CU_CORE_VERSION_PATCH;
}

inline std::string_view version_string() noexcept {
    return CU_CORE_VERSION_STRING;
}

inline std::string_view product_name() noexcept {
    return "CU_CORE";
}

inline std::string_view git_commit() noexcept {
    return CU_CORE_GIT_COMMIT;
}

inline std::string detailed() {
    std::string version(product_name());
    version.push_back(' ');
    version.append(version_string());
    constexpr std::string_view kUnknownCommit = "unknown";
    auto commit = git_commit();
    if (!commit.empty() && commit != kUnknownCommit) {
        version.append(" (");
        version.append(commit);
        version.push_back(')');
    }
    return version;
}

} // namespace version_info
