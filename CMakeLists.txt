cmake_minimum_required(VERSION 3.0)
project(quadmesh VERSION 0.1.0 LANGUAGES CXX)

###################################################################################################
# Include additional cmake packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
###################################################################################################

set(CMAKE_CXX_STANDARD 17)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

###################################################################################################
# Configure conan
if(NOT EXISTS ${CMAKE_BINARY_DIR}/conan.cmake)
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
                  "${CMAKE_BINARY_DIR}/conan.cmake")
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(
    REQUIRES gtest/1.8.1@bincrafters/stable
             fmt/5.2.1@bincrafters/stable
             spdlog/1.2.1@bincrafters/stable
    BASIC_SETUP CMAKE_TARGETS
    BUILD missing
)
###################################################################################################

###################################################################################################
# Declare dependencies
find_package(GeomModel REQUIRED)
find_package(StepParser REQUIRED)
###################################################################################################

###################################################################################################
# Find source files
file(GLOB MAIN_HEADERS "include/quadmesh/*.h")

set(MAIN_EXECUTABLE "src/main.cpp")
set(MAIN_SOURCES "")
file(GLOB _SRC_CHILDREN "src/*")

foreach(_CHILD ${_SRC_CHILDREN})
    if(IS_DIRECTORY ${_CHILD})
        file(GLOB _CHILD_HEADERS "${_CHILD}/*.h")
	    file(GLOB _CHILD_SOURCES "${_CHILD}/*.cpp")
        list(APPEND MAIN_SOURCES ${_CHILD_HEADERS})
        list(APPEND MAIN_SOURCES ${_CHILD_SOURCES})
	endif()
endforeach()
###################################################################################################

###################################################################################################
# Create target and set properties
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

add_library(libquadmesh SHARED ${MAIN_HEADERS} ${MAIN_SOURCES})

target_include_directories(libquadmesh
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CONAN_INCLUDE_DIRS_FMT}
        "src"
)

target_link_libraries(libquadmesh
    PUBLIC
        GeomModel::GeomModel
    PRIVATE
        CONAN_PKG::fmt
        CONAN_PKG::spdlog
)

add_executable(quadmesh ${MAIN_EXECUTABLE})

target_include_directories(quadmesh
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CONAN_INCLUDE_DIRS_FMT}
        "src"
)

target_link_libraries(quadmesh
    PUBLIC
        GeomModel::GeomModel
        libquadmesh
    PRIVATE
        StepParser::StepParser
        CONAN_PKG::fmt
        CONAN_PKG::spdlog
)

###################################################################################################
# Add tests
add_subdirectory(test)
###################################################################################################
