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
    REQUIRES
        gtest/1.8.1@bincrafters/stable
        fmt/5.2.1@bincrafters/stable
        spdlog/1.2.1@bincrafters/stable
        boost/1.69.0@conan/stable
    BASIC_SETUP CMAKE_TARGETS
    BUILD missing
    OPTIONS
        gtest:shared=False
        
        boost:without_math=True
        boost:without_wave=True
        boost:without_container=True
        boost:without_contract=True
        boost:without_exception=True
        boost:without_graph=True
        boost:without_iostreams=True
        boost:without_locale=True
        boost:without_log=True
        boost:without_program_options=False
        boost:without_random=True
        boost:without_regex=True
        boost:without_mpi=True
        boost:without_serialization=True
        boost:without_coroutine=True
        boost:without_fiber=True
        boost:without_context=True
        boost:without_timer=True
        boost:without_thread=True
        boost:without_chrono=True
        boost:without_date_time=True
        boost:without_atomic=True
        boost:without_filesystem=True
        boost:without_system=True
        boost:without_graph_parallel=True
        boost:without_python=True
        boost:without_stacktrace=True
        boost:without_test=True
        boost:without_type_erasure=True
          
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

add_library(libquadmesh STATIC ${MAIN_HEADERS} ${MAIN_SOURCES})

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
