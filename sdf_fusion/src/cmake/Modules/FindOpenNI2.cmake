# Find OpenNI 2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI 2 was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI 2 include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI 2.
# OPENNI2_DEFINITIONS - Compiler flags for OPENNI 2.
#

find_package(PkgConfig QUIET)

pkg_check_modules(PC_OPENNI2 QUIET libopenni2)

set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

set(OPENNI2_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OPENNI2_SUFFIX 64)
endif(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)

find_path(OPENNI2_INCLUDE_DIRS OpenNI.h
    PATHS
    "$ENV{OPENNI2_INCLUDE${OPENNI2_SUFFIX}}"  # Win64 needs '64' suffix
    /usr/include/openni2  # common path for deb packages
)

find_library(OPENNI2_LIBRARY
             NAMES OpenNI2  # No suffix needed on Win64
             libOpenNI2     # Linux
             PATHS "$ENV{OPENNI2_LIB${OPENNI2_SUFFIX}}"  # Windows default path, Win64 needs '64' suffix
             "$ENV{OPENNI2_REDIST}"                      # Linux install does not use a separate 'lib' directory
             )

set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG OPENNI2_LIBRARY OPENNI2_INCLUDE_DIRS)

mark_as_advanced(OPENNI2_LIBRARY OPENNI2_INCLUDE_DIRS)

if(OPENNI2_FOUND)
  # Add the include directories
  set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
  set(OPENNI2_REDIST_DIR $ENV{OPENNI2_REDIST${OPENNI2_SUFFIX}})
  message(STATUS "OpenNI 2 found (include: ${OPENNI2_INCLUDE_DIRS}, lib: ${OPENNI2_LIBRARY}, redist: ${OPENNI2_REDIST_DIR})")
endif(OPENNI2_FOUND)