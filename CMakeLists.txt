cmake_minimum_required(VERSION 3.18)

# Prebuilt distribution: this repository carries the SDK headers and
# committed static libraries, no SDK source. The library is selected per
# platform and architecture and all third-party dependencies (zenoh-c
# prebuilt, spdlog / zenoh-cpp headers) are vendored, so configure needs
# no network access.
#
# The version placeholder is substituted by ci/build_dist.py when the
# tree is assembled from the source repository; this file is not meant
# to be configured directly from payload/shidou_sdk/.
project(shidou_sdk VERSION 2.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SHIDOU_BUILD_EXAMPLES "Build example applications" ON)

# Processor names differ between hosts, generators and toolchain files
# (x86_64 / AMD64 / arm64 / aarch64), so normalize and compare for
# equality: a pattern match on "arm" would also accept armv7l.
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SHIDOU_PROC)

if(WIN32)
    if(SHIDOU_PROC STREQUAL "arm64" OR SHIDOU_PROC STREQUAL "aarch64")
        set(SHIDOU_PLATFORM win-arm64)
    else()
        set(SHIDOU_PLATFORM win)
    endif()
elseif(UNIX AND NOT APPLE)
    if(SHIDOU_PROC STREQUAL "x86_64" OR SHIDOU_PROC STREQUAL "amd64")
        set(SHIDOU_PLATFORM linux)
    elseif(SHIDOU_PROC STREQUAL "aarch64" OR SHIDOU_PROC STREQUAL "arm64")
        set(SHIDOU_PLATFORM linux-arm64)
    else()
        message(FATAL_ERROR
            "No prebuilt shidou_core for this processor: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
else()
    message(FATAL_ERROR "No prebuilt shidou_core for this platform.")
endif()
set(SHIDOU_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${SHIDOU_PLATFORM}")
# The Windows payload keeps its runtime binary in a directory of its own,
# per platform; the deploy function below copies it beside executables.
set(SHIDOU_ZENOHC_DLL "${CMAKE_CURRENT_SOURCE_DIR}/bin/${SHIDOU_PLATFORM}/zenohc.dll")

# zenoh-c: vendored prebuilt release. The target is recreated here
# (instead of find_package) because the official package config
# resolves paths relative to its own location, which does not survive
# the per-platform lib/{win,win-arm64,linux,linux-arm64} nesting of
# this repository. The properties mirror the official config's shared
# target.
if(NOT TARGET zenohc::lib)
    add_library(zenohc::lib SHARED IMPORTED GLOBAL)
    set_target_properties(zenohc::lib PROPERTIES
        INTERFACE_COMPILE_DEFINITIONS ZENOHC_DYN_LIB
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
    if(WIN32)
        set_target_properties(zenohc::lib PROPERTIES
            IMPORTED_LOCATION "${SHIDOU_ZENOHC_DLL}"
            IMPORTED_IMPLIB "${SHIDOU_LIB_DIR}/zenohc.dll.lib")
    else()
        # Both Linux platforms share this branch: the architecture is
        # separated by SHIDOU_LIB_DIR, not by the target layout.
        set_target_properties(zenohc::lib PROPERTIES
            IMPORTED_LOCATION "${SHIDOU_LIB_DIR}/libzenohc.so")
    endif()
endif()

# spdlog: vendored header-only, same interface as the source build.
# GLOBAL: consumer projects may link shidou::core from outside this
# directory, which needs every imported target in scope.
if(NOT TARGET spdlog::spdlog_header_only)
    add_library(spdlog::spdlog_header_only INTERFACE IMPORTED GLOBAL)
    set_target_properties(spdlog::spdlog_header_only PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
        INTERFACE_COMPILE_DEFINITIONS SPDLOG_HEADER_ONLY
        # fmt requires /utf-8 on MSVC.
        INTERFACE_COMPILE_OPTIONS "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:MSVC>>:/utf-8>")
endif()

# zenoh-cpp: vendored headers on top of the zenoh-c library.
if(NOT TARGET zenohcxx::zenohc)
    add_library(zenohcxx::zenohc INTERFACE IMPORTED GLOBAL)
    set_target_properties(zenohcxx::zenohc PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
        INTERFACE_LINK_LIBRARIES zenohc::lib
        INTERFACE_COMPILE_DEFINITIONS ZENOHCXX_ZENOHC)
endif()

# shidou_core: committed prebuilt static library (Release configuration).
if(WIN32)
    set(SHIDOU_CORE_LIB "${SHIDOU_LIB_DIR}/shidou_core.lib")
else()
    set(SHIDOU_CORE_LIB "${SHIDOU_LIB_DIR}/libshidou_core.a")
endif()
if(NOT EXISTS "${SHIDOU_CORE_LIB}")
    message(FATAL_ERROR "Prebuilt shidou_core missing: ${SHIDOU_CORE_LIB}")
endif()
# The Windows runtime binary lives in a directory of its own: a tree that
# carries lib/<platform>/ but no bin/<platform>/ fails here, while the
# path is still in hand, instead of at the first deploy step.
if(WIN32 AND NOT EXISTS "${SHIDOU_ZENOHC_DLL}")
    message(FATAL_ERROR "Prebuilt zenohc runtime missing: ${SHIDOU_ZENOHC_DLL}")
endif()
add_library(shidou::core STATIC IMPORTED GLOBAL)
set_target_properties(shidou::core PROPERTIES
    IMPORTED_LOCATION "${SHIDOU_CORE_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    INTERFACE_LINK_LIBRARIES "zenohcxx::zenohc;spdlog::spdlog_header_only"
    INTERFACE_COMPILE_FEATURES "cxx_std_17")

# Copy zenohc.dll next to executables on Windows so they can run.
function(shidou_deploy_zenohc_runtime target)
    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SHIDOU_ZENOHC_DLL}"
                $<TARGET_FILE_DIR:${target}>)
    endif()
endfunction()

# Examples: built against the prebuilt library to prove the committed
# package works end to end. The target list is injected at assembly time by
# ci/build_dist.py (from its EXAMPLE_TARGETS); the template names no example
# of its own.
if(SHIDOU_BUILD_EXAMPLES)
    foreach(name IN ITEMS chassis_control get_joint get_state mit_control pushrod_control set_fsm)
        add_executable(${name} examples/${name}.cpp)
        target_link_libraries(${name} PRIVATE shidou::core)
        shidou_deploy_zenohc_runtime(${name})
    endforeach()
endif()
