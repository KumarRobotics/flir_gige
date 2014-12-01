# FindEbus.cmake - Find ebus sdk, version >= 4.
# quchao@seas.upenn.edu (Chao Qu)
# Modified from FindEigen.cmake by alexs.mac@gmail.com  (Alex Stewart)
#
# This module defines the following variables:
#
# Ebus_FOUND:        TRUE if ebus is found.
# Ebus_INCLUDE_DIRS: Include directories for ebus.
# Ebus_LIBRARIES:    Libraries for all ebus component libraries
#                    and dependencies.
#
# Ebus_VERSION: Extracted from lib/PvBase.so.x.y.z
# Ebus_WORLD_VERSION: Equal to 4 if Ebus_VERSION = 4.0.5
# Ebus_MAJOR_VERSION: Equal to 0 if Ebus_VERSION = 4.0.5
# Ebus_MINOR_VERSION: Equal to 5 if Ebus_VERSION = 4.0.5
#
# The following variables control the behaviour of this module:
#
# Ebus_INCLUDE_DIR_HINTS: List of additional directories in which to
#                         search for ebus includes, e.g: /foo/include.
# Ebus_LIBRARY_DIR_HINTS: List of additional directories in which to
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
# Ebus_INCLUDE_DIR: Include directory for ebus, not including the
#                    include directory of any dependencies.
# Ebus_LIBRARY: ebus library, not including the libraries of any
#                dependencies.

# Called if we failed to find ebus or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(Ebus_REPORT_NOT_FOUND REASON_MSG)
    unset(Ebus_FOUND)
    unset(Ebus_INCLUDE_DIRS)
    unset(Ebus_LIBRARIES)
    unset(Ebus_WORLD_VERSION)
    unset(Ebus_MAJOR_VERSION)
    unset(Ebus_MINOR_VERSION)
    # Make results of search visible in the CMake GUI if ebus has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Ebus_INCLUDE_DIR)
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
endmacro(Ebus_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Ebus_CHECK_INCLUDE_DIRS
    /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/include)
list(APPEND Ebus_CHECK_LIBRARY_DIRS
    /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/lib)

# Check general hints
if(Ebus_HINTS AND EXISTS ${Ebus_HINTS})
    set(Ebus_INCLUDE_DIR_HINTS ${Ebus_HINTS}/include)
    set(Ebus_LIBRARY_DIR_HINTS ${Ebus_HINTS}/lib)
endif()

# Search supplied hint directories first if supplied.
# Find include directory for ebus
find_path(Ebus_INCLUDE_DIR
    NAMES PvBase.h
    PATHS ${Ebus_INCLUDE_DIR_HINTS}
    ${Ebus_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ebus_INCLUDE_DIR OR NOT EXISTS ${Ebus_INCLUDE_DIR})
    Ebus_REPORT_NOT_FOUND(
        "Could not find ebus include directory, set Ebus_INCLUDE_DIR to "
        "path to ebus include directory,"
        "e.g. /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/include.")
else()
    message(STATUS "ebus include dir found: " ${Ebus_INCLUDE_DIR})
endif()

# Find library directory for ebus
find_library(Ebus_LIBRARY
    NAMES PvBase
    PATHS ${Ebus_LIBRARY_DIR_HINTS}
    ${Ebus_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ebus_LIBRARY OR NOT EXISTS ${Ebus_LIBRARY})
    Ebus_REPORT_NOT_FOUND(
        "Could not find ebus library, set Ebus_LIBRARY "
        "to full path to ebus library direcotory.")
else()
    # TODO: need to fix this hacky solution for getting Ebus_LIBRARY_DIR
    string(REGEX MATCH ".*/" Ebus_LIBRARY_DIR ${Ebus_LIBRARY})
endif()

# Mark internally as found, then verify. Ebus_REPORT_NOT_FOUND() unsets if
# called.
set(Ebus_FOUND TRUE)

# Extract ebus version from ebus_sdk/Ubuntu-12.04-x86_64/lib/libPvBase.so.x.y.z
if(Ebus_LIBRARY_DIR)
    file(GLOB Ebus_PVBASE
        RELATIVE ${Ebus_LIBRARY_DIR}
        ${Ebus_LIBRARY_DIR}/libPvBase.so.[0-9].[0-9].[0-9])
    # TODO: add version support
    # string(REGEX MATCH ""
    #       Ebus_WORLD_VERSION ${Ebus_PVBASE})
    # message(STATUS "ebus world version: " ${Ebus_WORLD_VERSION})
endif()

# Catch case when caller has set Ebus_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(Ebus_INCLUDE_DIR AND NOT EXISTS ${Ebus_INCLUDE_DIR}/PvBase.h)
    Ebus_REPORT_NOT_FOUND("Caller defined Ebus_INCLUDE_DIR: "
        ${Ebus_INCLUDE_DIR}
        " does not contain PvBase.h header.")
endif()

# Set standard CMake FindPackage variables if found.
if(Ebus_FOUND)
    set(Ebus_INCLUDE_DIRS ${Ebus_INCLUDE_DIR})
    file(GLOB Ebus_LIBRARIES ${Ebus_LIBRARY_DIR}libPv*.so)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
if(Ebus_FOUND)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Ebus DEFAULT_MSG
        Ebus_INCLUDE_DIRS Ebus_LIBRARIES)
endif()

# Only mark internal variables as advanced if we found ebus, otherwise
# leave it visible in the standard GUI for the user to set manually.
if(Ebus_FOUND)
    mark_as_advanced(FORCE Ebus_INCLUDE_DIR Ebus_LIBRARY)
endif()
