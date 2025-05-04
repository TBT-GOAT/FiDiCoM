# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( random-spatial-network-planning )


# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


# Eigen
find_package(Eigen3 3.4.0)   

if (EIGEN3_FOUND)   
  include( ${EIGEN3_USE_FILE} ) 
endif()

# pagmo
find_package(pagmo REQUIRED)

# add subdirectory
add_subdirectory(app)
add_subdirectory(core)
add_subdirectory(domain)