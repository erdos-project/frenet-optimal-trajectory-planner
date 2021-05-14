cmake_minimum_required(VERSION 3.10)
project(FrenetOptimalTrajectory)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall -DQT_NO_VERSION_TAGGING")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(.)
include_directories(
        ${CUSTOM_EIGEN}
)

include_directories("${CMAKE_SOURCE_DIR}/src"
                    "${CMAKE_SOURCE_DIR}/src/CubicSpline"
                    "${CMAKE_SOURCE_DIR}/src/Polynomials"
                    "${CMAKE_SOURCE_DIR}/src/FrenetOptimalTrajectory"
                    "${CMAKE_SOURCE_DIR}/src/Obstacle"
                    "${CMAKE_SOURCE_DIR}/src/Car")

add_library(FrenetOptimalTrajectory SHARED
            # src/main.cpp
            src/utils.h
            src/Polynomials/QuarticPolynomial.cpp
            src/Polynomials/QuarticPolynomial.h
            src/Polynomials/QuinticPolynomial.cpp
            src/Polynomials/QuinticPolynomial.h
            src/CubicSpline/CubicSpline1D.cpp
            src/CubicSpline/CubicSpline1D.h
            src/CubicSpline/CubicSpline2D.cpp
            src/CubicSpline/CubicSpline2D.h
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/FrenetPath.cpp
            src/FrenetOptimalTrajectory/FrenetPath.h
            src/FrenetOptimalTrajectory/fot_wrapper.cpp
            src/FrenetOptimalTrajectory/py_cpp_struct.h
            src/Obstacle/Obstacle.cpp
            src/Obstacle/Obstacle.h
            src/Car/Car.cpp
            src/Car/Car.h)

add_library(PackageFrenetOptimalTrajectory STATIC
            # src/main.cpp
            src/utils.h
            src/Polynomials/QuarticPolynomial.cpp
            src/Polynomials/QuarticPolynomial.h
            src/Polynomials/QuinticPolynomial.cpp
            src/Polynomials/QuinticPolynomial.h
            src/CubicSpline/CubicSpline1D.cpp
            src/CubicSpline/CubicSpline1D.h
            src/CubicSpline/CubicSpline2D.cpp
            src/CubicSpline/CubicSpline2D.h
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/FrenetPath.cpp
            src/FrenetOptimalTrajectory/FrenetPath.h
            src/FrenetOptimalTrajectory/fot_wrapper.cpp
            src/FrenetOptimalTrajectory/py_cpp_struct.h
            src/Obstacle/Obstacle.cpp
            src/Obstacle/Obstacle.h
            src/Car/Car.cpp
            src/Car/Car.h)

add_executable(AnytimeFrenetOptimalTrajectoryTest
            src/anytime.cpp
            src/utils.h
            src/Polynomials/QuarticPolynomial.cpp
            src/Polynomials/QuarticPolynomial.h
            src/Polynomials/QuinticPolynomial.cpp
            src/Polynomials/QuinticPolynomial.h
            src/CubicSpline/CubicSpline1D.cpp
            src/CubicSpline/CubicSpline1D.h
            src/CubicSpline/CubicSpline2D.cpp
            src/CubicSpline/CubicSpline2D.h
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/FrenetPath.cpp
            src/FrenetOptimalTrajectory/FrenetPath.h
            src/FrenetOptimalTrajectory/fot_wrapper.cpp
            src/FrenetOptimalTrajectory/py_cpp_struct.h
            src/Obstacle/Obstacle.cpp
            src/Obstacle/Obstacle.h
            src/Car/Car.cpp
            src/Car/Car.h)

add_executable(FrenetOptimalTrajectoryTest
               src/main.cpp
               src/utils.h
               src/Polynomials/QuarticPolynomial.cpp
               src/Polynomials/QuarticPolynomial.h
               src/Polynomials/QuinticPolynomial.cpp
               src/Polynomials/QuinticPolynomial.h
               src/CubicSpline/CubicSpline1D.cpp
               src/CubicSpline/CubicSpline1D.h
               src/CubicSpline/CubicSpline2D.cpp
               src/CubicSpline/CubicSpline2D.h
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
               src/FrenetOptimalTrajectory/FrenetPath.cpp
               src/FrenetOptimalTrajectory/FrenetPath.h
               src/FrenetOptimalTrajectory/fot_wrapper.cpp
               src/FrenetOptimalTrajectory/py_cpp_struct.h
               src/Obstacle/Obstacle.cpp
               src/Obstacle/Obstacle.h
               src/Car/Car.cpp
               src/Car/Car.h)
target_link_libraries(FrenetOptimalTrajectory Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(PackageFrenetOptimalTrajectory Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(FrenetOptimalTrajectoryTest Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(AnytimeFrenetOptimalTrajectoryTest Eigen3::Eigen Qt5::Widgets Qt5::Gui)