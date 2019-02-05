cmake_minimum_required(VERSION 2.6)
enable_testing()
project(clamp CXX C)

# version indicator
set(CLAMP_VERSION_MAJOR 1)
set(CLAMP_VERSION_MINOR 0)
set(CLAMP_VERSION_PATCH 0)
set(CLAMP_VERSION_STRING "${CLAMP_VERSION_MAJOR}.${CLAMP_VERSION_MINOR}.${CLAMP_VERSION_PATCH}")


# option: whether turn on Matlab toolbox
option(CLAMP_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox" OFF)


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

# Find GPMP2
find_package(gpmp2 REQUIRED)
include_directories(${GPMP2_INCLUDE_DIR})
set(GPMP2_LIBRARIES gpmp2)

# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(clamp)


# Wrapping to MATLAB
if(CLAMP_BUILD_MATLAB_TOOLBOX)
  # wrap
  include(GtsamMatlabWrap)
  wrap_and_install_library(clamp.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
  
  # install matlab functions and scripts
  add_subdirectory(matlab)
endif()

# Install config and export files
GtsamMakeConfigFile(clamp)
export(TARGETS ${clamp_EXPORTED_TARGETS} FILE clamp-exports.cmake)
set_target_properties(clamp PROPERTIES LINKER_LANGUAGE CXX)
