cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(barabella)

# compiler options
add_definitions("-std=gnu++11")

# deal with pcl
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# deal with boost
find_package(Boost COMPONENTS signals filesystem serialization)
include_directories(${Boost_INCLUDE_DIR} )


### configure ###
set (BARABELLA_VERSION 0.1)
set (BB_DEBUG_LVL 4)

configure_file (
    "${PROJECT_SOURCE_DIR}/src/core/barabella_config.h.in"
    "${PROJECT_SOURCE_DIR}/src/core/barabella_config.h"
)
#################

add_subdirectory(src)
