# Findebus.cmake - Find ebus sdk, version >= 4.
# Modified from FindEigen.cmake by alexs.mac@gmail.com  (Alex Stewart)
#
# This module defines the following variables:
#
# EBUS_FOUND: TRUE if ebus is found.
# EBUS_INCLUDE_DIRS: Include directories for ebus.
# EBUS_LIBRARIES: Libraries for all ebus component libraries and dependencies.
#
# EBUS_VERSION: Extracted from lib/PvBase.so.x.y.z
# EBUS_WORLD_VERSION: Equal to 4 if EBUS_VERSION = 4.0.5
# EBUS_MAJOR_VERSION: Equal to 0 if EBUS_VERSION = 4.0.5
# EBUS_MINOR_VERSION: Equal to 5 if EBUS_VERSION = 4.0.5
#
# The following variables control the behaviour of this module:
#
# EBUS_INCLUDE_DIR_HINTS: List of additional directories in which to
#                         search for ebus includes, e.g: /foo/include.
# EBUS_LIBRARY_DIR_HINTS: List of additional directories in which to
#                         search for ebus libraries, e.g: /bar/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# EBUS_INCLUDE_DIR: Include directory for ebus, not including the
#                    include directory of any dependencies.
# EBUS_LIBRARY: ebus library, not including the libraries of any
#                dependencies.

# Called if we failed to find ebus or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(EBUS_REPORT_NOT_FOUND REASON_MSG)
    unset(EBUS_FOUND)
    unset(EBUS_INCLUDE_DIRS)
    unset(EBUS_LIBRARIES)
    unset(EBUS_WORLD_VERSION)
    unset(EBUS_MAJOR_VERSION)
    unset(EBUS_MINOR_VERSION)
    # Make results of search visible in the CMake GUI if ebus has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR EBUS_INCLUDE_DIR)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Ebus_FIND_QUIETLY)
        message(STATUS "Failed to find ebus - " ${REASON_MSG} ${ARGN})
    elseif(Ebus_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find ebus - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find ebus - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(EBUS_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND EBUS_CHECK_INCLUDE_DIRS
    /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/include)
list(APPEND EBUS_CHECK_LIBRARY_DIRS
    /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/lib)

# Check general hints
if(EBUS_HINTS AND EXISTS ${EBUS_HINTS})
    set(EBUS_INCLUDE_DIR_HINTS ${EBUS_HINTS}/include)
    set(EBUS_LIBRARY_DIR_HINTS ${EBUS_HINTS}/lib)
endif()

# Search supplied hint directories first if supplied.
# Find include directory for ebus
find_path(EBUS_INCLUDE_DIR
    NAMES PvBase.h
    PATHS ${EBUS_INCLUDE_DIR_HINTS}
    ${EBUS_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT EBUS_INCLUDE_DIR OR NOT EXISTS ${EBUS_INCLUDE_DIR})
    EBUS_REPORT_NOT_FOUND(
        "Could not find ebus include directory, set EBUS_INCLUDE_DIR to "
        "path to ebus include directory,"
        "e.g. /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/include.")
else()
    message(STATUS "ebus include dir found: " ${EBUS_INCLUDE_DIR})
endif()

# Find library directory for ebus
find_library(EBUS_LIBRARY
    NAMES PvBase
    PATHS ${EBUS_LIBRARY_DIR_HINTS}
    ${EBUS_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT EBUS_LIBRARY OR NOT EXISTS ${EBUS_LIBRARY})
    EBUS_REPORT_NOT_FOUND(
        "Could not find ebus library, set EBUS_LIBRARY "
        "to full path to ebus library direcotory.")
else()
    # TODO: need to fix this hacky solution for getting EBUS_LIBRARY_DIR
    string(REGEX MATCH ".*/" EBUS_LIBRARY_DIR ${EBUS_LIBRARY})
endif()

# Mark internally as found, then verify. EBUS_REPORT_NOT_FOUND() unsets if
# called.
set(EBUS_FOUND TRUE)

# Extract ebus version from ebus_sdk/Ubuntu-12.04-x86_64/lib/libPvBase.so.x.y.z
if(EBUS_LIBRARY_DIR)
    file(GLOB EBUS_PVBASE
        RELATIVE ${EBUS_LIBRARY_DIR}
        ${EBUS_LIBRARY_DIR}/libPvBase.so.[0-9].[0-9].[0-9])
    # TODO: add version support
    # string(REGEX MATCH ""
    #       EBUS_WORLD_VERSION ${EBUS_PVBASE})
    # message(STATUS "ebus world version: " ${EBUS_WORLD_VERSION})
endif()

# Catch case when caller has set EBUS_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(EBUS_INCLUDE_DIR AND NOT EXISTS ${EBUS_INCLUDE_DIR}/PvBase.h)
    EBUS_REPORT_NOT_FOUND("Caller defined EBUS_INCLUDE_DIR: "
        ${EBUS_INCLUDE_DIR}
        " does not contain PvBase.h header.")
endif()

# Set standard CMake FindPackage variables if found.
if(EBUS_FOUND)
    set(EBUS_INCLUDE_DIRS ${EBUS_INCLUDE_DIR})
    file(GLOB EBUS_LIBRARIES ${EBUS_LIBRARY_DIR}libPv*.so)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
if(EBUS_FOUND)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Ebus DEFAULT_MSG
        EBUS_INCLUDE_DIRS EBUS_LIBRARIES)
endif()

# Only mark internal variables as advanced if we found ebus, otherwise
# leave it visible in the standard GUI for the user to set manually.
if(EBUS_FOUND)
    mark_as_advanced(FORCE EBUS_INCLUDE_DIR EBUS_LIBRARY)
endif()
